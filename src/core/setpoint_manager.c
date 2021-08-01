/**
* @file setpoint_manager.c
*
*
**/
#include <stdio.h>
#include <math.h>
#include <string.h> // for memset

#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <rc/start_stop.h>
#include <rc/math/quaternion.h>

#include <input_manager.h>
#include <servos.h>
#include <setpoint_manager.h>
#include <settings.h>
#include <feedback.h>
#include <state_estimator.h>
#include <rcs_defs.h>
#include <flight_mode.h>
#include <tools.h>

setpoint_t setpoint; // extern variable in setpoint_manager.h
flight_status_t flight_status;
events_t events;

void __update_ap(void)
{
	/*
	This function should be called to update the altitude controller command.
	Note that we are trying to limit the error input to the feedback controllers,
	therefore we need to avoid values above a certain limit. Even if the target apogee is 
	fixed, controller will see error in the range of ALT_MAX_ERROR.
	We would theoretically want to make controller not do anything if projected apogee
	is bellow the target since we don't have control over propulsion system, but this should 
	not be done within this funtion and be a dedicated flight mode (CRUISE or DESCENT)

	setpoint.alt is one of the inputs to the altitude controller in feedback.c 
	that directly effects motion allong body-fixed X-axis which is
	in the direction of flight of the rocket vehicle. See mix.h, mix.c, feedback.c for more
	details.
	*/

	// make sure setpoint doesn't go too far to avoid controllers going crazy
	if (state_estimate.proj_ap > (settings.target_altitude_m + ALT_MAX_ERROR)) {
		setpoint.alt = settings.target_altitude_m + ALT_MAX_ERROR;//if above target altitude
		return;
	}
	else if (state_estimate.proj_ap < (settings.target_altitude_m - ALT_MAX_ERROR)) {
		setpoint.alt = settings.target_altitude_m - ALT_MAX_ERROR; //if below target altitude
		return;
	}
	else {
		setpoint.alt = state_estimate.proj_ap; //dont limit the error
	}
	
	return;
}

int setpoint_manager_init(void)
{
	if(setpoint.initialized){
		fprintf(stderr, "ERROR in setpoint_manager_init, already initialized\n");
		return -1;
	}
	memset(&setpoint,0,sizeof(setpoint_t));

	//Need these for __flight_status_update()
	flight_status		= WAIT;
	events.burnout_fl	= 0;
	events.ignition_fl	= 0;

	
	user_input.flight_mode	= IDLE;
	setpoint.init_time		= rc_nanos_since_boot();
	setpoint.initialized	= 1;
	return 0;
}

/* __flight_status_update
*
* this function updates the flight status
*
* returns> 0 if sucessfull -1 on failure
*/

/*
TODO:
- verify all tolerances and logic
- add logic for more advanced event detection:
	- for separation
	- pressure-based sensor failure analisys (cross-check chages in pressure to imu and gyro data test if the sensors are working) - may need to be implemented elsewhere
	- restart failsafe - make sure the event logic can be easily restarted in case of failure (possibly in flight? make it safe to do so...)
*/
int __flight_status_update(void)
{
	//Check for tipover:
	if (events.tipover_detected)
	{
		//printf("TIPOVER DETECTED \n");
		user_input.flight_mode = IDLE; //disable controllers
		servos_return_to_nominal(); //return servos to nominal
	}

	//always keep the highest altitude as apogee altitude (UNPOWERED_ASCENT relies on this)
	if (events.apogee_alt < state_estimate.alt_bmp) 
	{
		events.apogee_alt	= state_estimate.alt_bmp;
		events.apogee_fl	= 0; //reset apogee flag
	}
	
	//always update the current altitude for landing if it has escaped outside the tol.
	if (events.land_alt > state_estimate.alt_bmp + settings.event_landing_alt_tol 
		|| events.land_alt < state_estimate.alt_bmp - settings.event_landing_alt_tol)
	{
		events.land_alt		= state_estimate.alt_bmp;
		events.land_fl		= 0; //still descending
		events.land_fl_vel	= 0;
	}
		


	//Check flight status:
	if (fstate.arm_state == DISARMED) //DISARMED and waiting for ARM command (servos are powered off)
	{
		/*
		events.ground_alt		= 0.0;
		events.apogee_alt		= 0.0;
		events.ignition_alt		= 0.0;
		events.burnout_alt		= 0.0;
		events.land_alt			= 0.0;
		*/
		flight_status			= WAIT;
		user_input.flight_mode	= IDLE; //shoud never switch modes on its own while disarmed
		return 0;
	}
	else //if armed - start checking for events
	{
		if (flight_status == WAIT) //just got ARMED
		{
			//make sure everything is armed and ready if arming was requested (should never really trigger this)
			if (user_input.requested_arm_mode == ARMED) {
				if (fstate.arm_state == DISARMED) feedback_arm();
				if (sstate.arm_state == DISARMED) servos_arm();
			}

			//This should happen once the system just got armed (on the launchpad)
			events.ground_alt	= state_estimate.alt_bmp;
			events.apogee_alt	= state_estimate.alt_bmp; //initialize apogee alt

			flight_status = STANDBY; //switch to the next flight status
			return 0;
		}
		else if (flight_status == STANDBY) //ARMED and waiting for IGNITION
		{
			if (events.ignition_fl != 1 && fabs(state_estimate.alt_bmp_accel) >= settings.event_launch_accel 
				&& fabs(state_estimate.alt_bmp - events.ground_alt) >= settings.event_launch_dh)
			{
				//Detected ignition
				events.init_time	= rc_nanos_since_boot();
				events.ignition_alt = state_estimate.alt_bmp;
				events.ignition_fl	= 1; //sensors have shown high accel and change in alt (can be noise)

				//don't accept motor ignition just yet
				return 0;
			}
			else if (events.ignition_fl && finddt_s(events.init_time) >= settings.event_ignition_delay_s)
			{
				//NOTE: check if ignition altitude is needed as a secondary condition
				// 
				//check if we are still accelerating and height has changed since detetection
				if (fabs(state_estimate.alt_bmp_accel) >= settings.event_launch_accel && fabs(state_estimate.alt_bmp - events.ground_alt) >= settings.event_launch_dh && fabs(state_estimate.alt_bmp - events.ignition_alt) >= settings.event_ignition_dh)
				{
					//&& fabs(state_estimate.alt_bmp - events.ignition_alt) >= settings.event_launch_dh
					//accept the fact the motor is burning now
					flight_status	= POWERED_ASCENT;
					events.meco_fl	= 0; //just in case, reset the MECO flag
					return 0;
				}
				else
				{
					//no ignition has been detected yet
					//reset flags if the ignition has not been confirmed
					events.ignition_fl	= 0;
					//flight_status		= STANDBY;
					return 0;
				}
				return -1;
			}
			else if (events.ignition_fl && fabs(state_estimate.alt_bmp_accel) >= settings.event_launch_accel 
				&& fabs(state_estimate.alt_bmp - events.ground_alt) >= settings.event_launch_dh)
			{
				events.ignition_fl = 1;
				return 0;
			}
			else
			{
				//no ignition has been detected yet
				//reset flags if the ignition has not been confirmed
				events.ignition_fl	= 0;
				//flight_status		= STANDBY;
				return 0;
			}
			return -1;
		}
		else if (flight_status == POWERED_ASCENT) //motor is burning and we can't do anything about it
		{
			//always keep the highest altitude as apogee altitude (UNPOWERED_ASCENT has its own apogee detection scheme)
			if (events.apogee_alt < state_estimate.alt_bmp && flight_status != UNPOWERED_ASCENT) events.apogee_alt = state_estimate.alt_bmp;

			if (events.ignition_fl != 1) printf("\n WARNING: POWERED_ASCENT triggered without ignition_fl == 1");

			//need to detect motor burnout:
			if (events.meco_fl != 1 && state_estimate.alt_bmp_vel > 0.0 && state_estimate.alt_bmp_accel < 0.0)
			{
				events.meco_fl = 1; //main engine cutoff detected
				events.init_time = rc_nanos_since_boot();
				return 0;
			}
			else if (events.meco_fl && finddt_s(events.init_time) >= settings.event_cutoff_delay_s )
			{
				if (fabs(state_estimate.alt_bmp - events.ground_alt)  >= settings.event_cutoff_dh && state_estimate.alt_bmp_accel <= 0.0)
				{
					flight_status		= UNPOWERED_ASCENT; //should be safe to proceed now
					events.apogee_fl	= 0;				//reset apogee flag just in case
					return 0;
				}
				else
				{
					//flight_status	= POWERED_ASCENT;
					events.meco_fl = 0; //main engine cutoff not detected (false alarm)
					return 0;
				}
				return -1;
			}
			else if (events.meco_fl && state_estimate.alt_bmp_vel > 0.0 && state_estimate.alt_bmp_accel < 0.0)
			{
				events.meco_fl = 1;
				return 0;
			}
			else
			{
				//reset flags if MECO has not been confirmed
				//flight_status	= POWERED_ASCENT;
				events.meco_fl	= 0; //main engine cutoff not detected
				return 0;
			}
			return -1;
		}
		else if (flight_status == UNPOWERED_ASCENT) // updated, has not been verified yet, Jack 4/2/2021
		{
			//finally! here's when we can switch the flight mode to apogee control and do any active control
			if (events.tipover_detected != 1)
			{
				user_input.flight_mode = AP_CTRL; //keep apogee control always active
			}

			//we have to detect apogee:
			//always keep the highest altitude as apogee altitude
			if (events.apogee_fl != 1 && events.apogee_alt > state_estimate.alt_bmp) //check if altitude has decreased for the first time
			{
				//events.apogee_alt	= state_estimate.alt_bmp; //set apogee to the current alt
				events.init_time	= rc_nanos_since_boot();
				events.apogee_fl	= 1;
				return 0;
			}
			//else if (events.apogee_fl && events.apogee_alt > state_estimate.alt_bmp) //check if altitude is still below apogee
			//{
			//	events.apogee_fl	= 0; //false alarm, reset flag
			//	events.apogee_alt	= state_estimate.alt_bmp; //set apogee to the current alt
			//	return 0;
			//}
			else if (events.apogee_fl && finddt_s(events.init_time) >= settings.event_apogee_delay_s) //if no increase in apogee for a few milliseconds
			{
				if (state_estimate.alt_bmp_vel <= 0.0 && fabs(state_estimate.alt_bmp_accel) < settings.event_apogee_accel_tol)
				{
					// this is an early trigger, which should work if velocity is estimated properly
					flight_status	= DESCENT_TO_LAND;
					//pre-set everything for DESCENT_TO_LAND
					events.land_fl		= 0;
					events.land_fl_vel	= 0;
					
					//we have already passed apogee, so don't overwrite apogee altitude!
					events.land_alt = state_estimate.alt_bmp;
					return 0;
				}

				//will only reach this line if velocity is not estimated properly
				//wait more to confirm apogee based of altitude change

				if ( fabs(events.apogee_alt - state_estimate.alt_bmp) > settings.event_apogee_dh)
				{
					// this is a late failsafe to safeguard against velocity estimation failure
					flight_status	= DESCENT_TO_LAND;
					//pre-set everything for DESCENT_TO_LAND
					events.land_fl		= 0;
					events.land_fl_vel	= 0;
					
					events.land_alt		= state_estimate.alt_bmp; //we have already passed apogee for sure, so don't overwrite apogee altitude!
					return 0;
				}

				//none of the previous conditions have been trigered yet, just skip to the next run
				return 0;
			}
			else if (events.apogee_fl)
			{
				//apogee is not increasing, but we haven't confirmed it yet.
				return 0;
			}
			else
			{
				// apogee has not been detected - reset flag
				events.apogee_fl = 0;
				return 0;
			}
			return -1;
		}
		else if (flight_status == DESCENT_TO_LAND) //has not been verified yet, Jack 4/2/2021
		{
			//mission is almost over, we try to keep the system safe untill recovery
			user_input.flight_mode = IDLE; //disable all controllers
			servos_return_to_nominal();

			// Should we deactivate servos after some time? - may prevent from ground damage... 
			// TODO: We need to figure out a safe way to detect low altitude to deactivate servos 
			// before we actually touch the ground
			// ideally we would want to detect separations and deployments of the main before reaching 
			// critically low altitude
			if (state_estimate.alt_bmp < events.ground_alt + settings.event_start_landing_alt_m)
			{
				if (sstate.arm_state == ARMED) servos_disarm();
			}

			//landing detection is not critical, since the mission is over at this point
			//we can remain in DESCENT_TO_LAND safely because controllers are in IDLE mode (no control)

			// check if landed already, rocket on the ground would have close to zero change in altitude and very small velocity
			// use tolerances to account for small drift and numeric/sensor noise
			if (fabs(state_estimate.alt_bmp_accel) < settings.event_landning_accel_tol 
				&& fabs(state_estimate.alt_bmp - events.land_alt) < settings.event_landing_alt_tol 
				&& state_estimate.alt_bmp < events.ground_alt + settings.event_start_landing_alt_m) //if on the ground, altitude won't change much
			{
				//quick check, assume velocity is estimated correctly
				//note, landing under parachute is slow, below 30m/s, but will rarely be slower then 5 m/s (check with recovery)
				if (events.land_fl_vel != 1 && fabs(state_estimate.alt_bmp_vel) < settings.event_landing_vel_tol) //velocity should be very close to zero
				{
					//this is the begining of the fast landing detection algorithm (based on both alt and velocity)
					events.init_time	= rc_nanos_since_boot(); //record time
					events.land_fl_vel	= 1; //may have landed - need to verify
					return 0;
				}
				else if (events.land_fl_vel && fabs(state_estimate.alt_bmp_vel) < settings.event_landing_vel_tol)  //velocity should be very close to zero
				{
					if (finddt_s(events.init_time) >= settings.event_landing_delay_early_s) //confirm if enough time has passed (should be at least 5 seconds)
					{
						flight_status = LANDED;
						return 0;
					}
					else
					{
						events.land_fl_vel = 1; //may have landed - need to verify (make sure this is still enabled)
						return 0;
					}
					return -1;
				}
				else if (events.land_fl != 1)
				{
					//we need to safeguard against velocity estimation failure if the altitude has not changed for a while (altitude does not drift)
					//this is the begining of the slow landing detection algorithm (only based of the alt)
					//the algorithm will only reach this line if:
					// - altitude change is within tolerance (small, not moving vertically to much) 
					// - velocity estimation went haywire and it totally off (very large in magnitude, can not come back to zero)
					events.init_time_landed = rc_nanos_since_boot(); //record time
					events.land_fl			= 1;
					return 0;
				}
				else if (events.land_fl)
				{
					if (finddt_s(events.init_time_landed) >= settings.event_landing_delay_late_s) //confirm if enough time has passed (should be at least 20 seconds)
					{
						flight_status = LANDED;
						return 0;
					}
					else
					{
						//events.land_fl = 1; //may have landed - need to verify (make sure this is still enabled)
						return 0;
					}
					return -1;
				}
				else
				{
					//should never reach this line under normal conditions
					events.land_fl		= 0;
					events.land_fl_vel	= 0;
					return 0;
				}

				//should never reach this line
				return -1;

			}
			else //if altitude changes are more than the tolerance
			{
				/*
				if (events.land_alt < state_estimate.alt_bmp) //will normally happen when we are still descending
				{
					events.land_alt		= state_estimate.alt_bmp;
					events.land_fl		= 0;
					events.land_fl_vel	= 0;
					return 0;
				}
				else //what if not?
				{
					//Houston we have a problem -- need a failsafe here

					// will only reach here if altitude is not decreasing and previous conditions have not been triggered
					// - may indicate too tight tolerances on the above conditions 
					// - may have some logic error in the above statements
					// - sensor failure (barometer, specifically) - kinda late to the party if this is the case
					return 0;
				}
				*/

				//it should not be possible to reach this line since the altitude was already checked earlier, but just in case:
				events.land_fl = 0;
				events.land_fl_vel = 0;

				return 0;

			}
			return -1;
		}
		else if (flight_status == LANDED)
		{
			//don't do anything, make sure controllers are in iddle mode and servos are disarmed
			user_input.flight_mode			= IDLE;
			user_input.requested_arm_mode	= DISARMED; //should already be disarmed by this point, so just double check
			return 0;
		}
		else
		{
			if (flight_status == TEST)
			{
				//printf("Going into testing... \n");
				user_input.flight_mode = YP_TEST;
				return 0;
			}

			printf("ERROR in __flight_status_update, unknown flight status\n");
			return -1;
		}
		return -1;
	}
	return -1; //something went wrong - we should never reach this line
}



int setpoint_manager_update(void)
{
	if(setpoint.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, not initialized yet\n");
		return -1;
	}

	if(user_input.initialized==0){
		fprintf(stderr, "ERROR in setpoint_manager_update, input_manager not initialized yet\n");
		return -1;
	}

	
	if (__flight_status_update() != 0) {
		fprintf(stderr, "ERROR in __flight_status_update\n");
		return -1;
	}

	if (start_pre_flight_checks() == -1) printf("ERROR: Failed to start pre-flight checks\n");
	
	//for testing RPY controllers and actuators:
    /*
	if (finddt_s(setpoint.init_time) > 10 && finddt_s(setpoint.init_time) < 20)
	{
		user_input.requested_arm_mode = ARMED;
	}
	if (finddt_s(setpoint.init_time) > 20 && finddt_s(setpoint.init_time) < 60)
	{
		//user_input.requested_arm_mode = ARMED;
		user_input.flight_mode = YP_TEST;
		//user_input.flight_mode = AP_CTRL;
		//flight_status = TEST;
	}
	
	if (finddt_s(setpoint.init_time) > 60)
	{
		user_input.requested_arm_mode = DISARMED;
	}
	*/

	// if PAUSED or UNINITIALIZED, do nothing
	if(rc_get_state()!=RUNNING) return 0;

	// shutdown feedback and servos on kill switch
	if(user_input.requested_arm_mode == DISARMED){
		if(fstate.arm_state != DISARMED) feedback_disarm();
		if(sstate.arm_state != DISARMED) servos_disarm();
		return 0;
	}

	// finally, switch between flight modes and adjust setpoint properly
	switch(user_input.flight_mode){
	case IDLE:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 0;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 0;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;
		setpoint.alt	= 0;
		
		break;
	case AP_CTRL:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 1;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 0;

		setpoint.roll = 0;
		setpoint.pitch = 0;
		setpoint.yaw = 0;
		
		__update_ap();
		break;

	case YP_TEST:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 0; 
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 1;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;

		break;

	case YP_STABILIZE_AP:
		// configure which controllers are enabled
		setpoint.en_alt_ctrl	= 1;
		setpoint.en_r_ctrl		= 0;
		setpoint.en_py_ctrl		= 1;

		setpoint.roll	= 0;
		setpoint.pitch	= 0;
		setpoint.yaw	= 0;

		__update_ap();
		//TODO: implement limits on attitude control
		break;

	default: // should never get here
		fprintf(stderr,"ERROR in setpoint_manager thread, unknown flight mode\n");
		break;

	} // end switch(user_input.flight_mode)

	// arm feedback and servos when requested
	if(user_input.requested_arm_mode == ARMED){
		if(fstate.arm_state == DISARMED) feedback_arm();
		if(sstate.arm_state == DISARMED) servos_arm();
	}


	return 0;
}


int setpoint_manager_cleanup(void)
{
	setpoint.initialized=0;
	return 0;
}
