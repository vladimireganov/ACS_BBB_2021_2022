/**
 * @file feedback.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/time.h>

#include <feedback.h>
#include <rcs_defs.h>
#include <servos.h>
#include <thrust_map.h>
#include <mix.h>
#include <settings.h>
#include <state_estimator.h>
#include <xbee_packet_t.h>
#include <setpoint_manager.h>
#include <log_manager.h>

#define TWO_PI (M_PI*2.0)

feedback_state_t fstate; // extern variable in feedback.h

// keep original controller gains for scaling later
static double D_roll_gain_orig, D_pitch_gain_orig, D_yaw_gain_orig, D_X_gain_orig;

// filters
static rc_filter_t D_roll	= RC_FILTER_INITIALIZER;
static rc_filter_t D_pitch	= RC_FILTER_INITIALIZER;
static rc_filter_t D_yaw	= RC_FILTER_INITIALIZER;
static rc_filter_t D_X		= RC_FILTER_INITIALIZER;


static void __rpy_init(void)
{
	// get controllers from settings

	rc_filter_duplicate(&D_roll, settings.roll_controller);
	rc_filter_duplicate(&D_pitch, settings.pitch_controller);
	rc_filter_duplicate(&D_yaw, settings.yaw_controller);

#ifdef DEBUG
	printf("ROLL CONTROLLER:\n");
	rc_filter_print(D_roll);
	printf("PITCH CONTROLLER:\n");
	rc_filter_print(D_pitch);
	printf("YAW CONTROLLER:\n");
	rc_filter_print(D_yaw);
#endif

	// save original gains as we will scale these by battery voltage later
	D_roll_gain_orig	= D_roll.gain;
	D_pitch_gain_orig	= D_pitch.gain;
	D_yaw_gain_orig		= D_yaw.gain;


	// enable saturation. these limits will be changed late but we need to
	// enable now so that soft start can also be enabled
	rc_filter_enable_saturation(&D_roll, -MAX_ROLL_COMPONENT, MAX_ROLL_COMPONENT);
	rc_filter_enable_saturation(&D_pitch, -MAX_PITCH_COMPONENT, MAX_PITCH_COMPONENT);
	rc_filter_enable_saturation(&D_yaw, -MAX_YAW_COMPONENT, MAX_YAW_COMPONENT);
	// enable soft start
	rc_filter_enable_soft_start(&D_roll, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_pitch, SOFT_START_SECONDS);
	rc_filter_enable_soft_start(&D_yaw, SOFT_START_SECONDS);
}


int feedback_disarm(void)
{
	fstate.arm_state = DISARMED;
	// set LEDs
	rc_led_set(RC_LED_RED, 1);
	rc_led_set(RC_LED_GREEN, 0);
	return 0;
}

int feedback_arm(void)
{
	if (fstate.arm_state == ARMED) {
		printf("WARNING: trying to arm when controller is already armed\n");
		return -1;
	}
	// start a new log file every time controller is armed, this may take some
	// time so do it before touching anything else
	if (settings.enable_logging) log_manager_init();
	// get the current time
	fstate.arm_time_ns = rc_nanos_since_boot();
	// reset the index
	fstate.loop_index = 0;
	
	//static int last_en_alt_ctrl = 0; //make sure altitude control will go through initialization

	// zero out all filters
	rc_filter_reset(&D_roll);
	rc_filter_reset(&D_pitch);
	rc_filter_reset(&D_yaw);
	rc_filter_reset(&D_X);

	// prefill filters with current error
	//rc_filter_prefill_inputs(&D_roll, -state_estimate.roll);
	rc_filter_prefill_inputs(&D_pitch, -state_estimate.pitch);
	rc_filter_prefill_inputs(&D_yaw, -state_estimate.yaw);
	// set LEDs
	rc_led_set(RC_LED_RED, 0);
	rc_led_set(RC_LED_GREEN, 1);
	// last thing is to flag as armed
	fstate.arm_state = ARMED;
	return 0;
}



int feedback_init(void)
{

	__rpy_init();		// roll, pitch yaw feedback initializer

	rc_filter_duplicate(&D_X, settings.altitude_controller);


#ifdef DEBUG
	printf("ALTITUDE CONTROLLER:\n");
	rc_filter_print(D_X);
#endif

	D_X_gain_orig = D_X.gain;

	rc_filter_enable_saturation(&D_X, -1.0, 1.0);
	rc_filter_enable_soft_start(&D_X, SOFT_START_SECONDS);


	// make sure everything is disarmed them start the ISR
	feedback_disarm();

	fstate.initialized = 1;

	return 0;
}

int feedback_march(void)
{
	int i;
	double min, max;
	double u[MAX_INPUTS], mot[MAX_ROTORS];

	// declare altitude control flag
	static int last_en_alt_ctrl = 0; //0 if alt.control was not running last time 

	// Disarm if rc_state is somehow paused without disarming the controller.
	// This shouldn't happen if other threads are working properly.
	if (rc_get_state() != RUNNING && fstate.arm_state == ARMED) {
		feedback_disarm();
		printf("\n rc_state is somehow paused \n");
	}

	// Very important for safety! We need to check the deflection from vertical flight
	/*
	Note: this can be used later to activate attitude control.

	Ideal attitude correction sequence:
	1 - record the attitude (pitch and yaw) at the launch rail - this is our "true" flight path
	2 - record the attitude after burnout - this + some small tolerance is our limit on attitude deviation (don't go beyod this value pitch, yaw)
	3 - always check for tipover angle, make sure the ACS is not making the rocket go sideways (if one of the servos/fins breakoff)
		3.1 if error in yaw or pitch is too big - assume something went wrong in control (possible servo failure, fin breaking off)
			- disable control on a specific axis. if too much yaw - disable controllers along z/yaw axis
			don't kill power to servo rail, send pulse "return to nominal position" to servos. This will minimize asymetric drag that caused too much pitch/yaw
		3.2 if error in yaw and pitch is too big - assume something went wrong in control
			- same as for pitch/yaw error. Just use the same algotirhm for both channels 
	*/
	// check for attitude deviation:
	if (fabs(state_estimate.yaw) > TIP_ANGLE || fabs(state_estimate.pitch) > TIP_ANGLE) {
		events.tipover_detected = 1;

		//currenty this disables all control if there is too much yaw or pitch (rotation about y and z if x is in the direction of the nosecone)
		//check setpoint_mannager for cutoff sequence
	}
	else
	{
		events.tipover_detected = 0;
	}
	

	// if not running or not armed, keep the motors in an idle state
	if (rc_get_state() != RUNNING || fstate.arm_state == DISARMED) {
		//don't do anything. Since motors are not armed, there is not power to the servo rail
		return 0;
	}
	
	// We are about to start marching the individual SISO controllers forward.
	// Start by zeroing out the motors signals then add from there.
	for (i = 0; i < MAX_ROTORS; i++) mot[i] = 0.0;
	for (i = 0; i < MAX_INPUTS; i++) u[i] = 0.0;
	
	/***************************************************************************
	* Altitude Controller
	* run only if enabled
	***************************************************************************/
	if (setpoint.en_alt_ctrl == 0) last_en_alt_ctrl = 0; //make sure the flag is off

	if (setpoint.en_alt_ctrl)
	{
		//Run only during the first cycle (the first time step after altitude controll is on)
		if (last_en_alt_ctrl == 0)
		{

			rc_filter_reset(&D_X);   // reset the filter and reads from json
			
			rc_filter_prefill_outputs(&D_X, 0);
			last_en_alt_ctrl = 1;
		}


		mix_check_saturation(VEC_X, mot, &min, &max);
		if (max > MAX_X_COMPONENT)  max = MAX_X_COMPONENT;
		if (min < -MAX_X_COMPONENT) min = -MAX_X_COMPONENT;
		if (max < 0.01 * MAX_X_COMPONENT && min > -0.01 * MAX_X_COMPONENT) {
			//printf("WARNING: controller bounds are zeros, changing them to to MAX_X_COMPONENT...\n");
			max = MAX_X_COMPONENT;
			min = -MAX_X_COMPONENT;
		}
		rc_filter_enable_saturation(&D_X, min, max);
		D_X.gain = D_X_gain_orig * settings.v_nominal / state_estimate.v_batt_lp; //updating the gains based on battery voltage
		//u[VEC_X] = rc_filter_march(&D_X, (settings.target_altitude_m - setpoint.alt)); //this has to be the error between target and predicted value
		u[VEC_X] = rc_filter_march(&D_X, (settings.target_altitude_m - setpoint.alt)/ALT_MAX_ERROR); //this has to be the error between target and predicted value
		mix_add_input(u[VEC_X], VEC_X, mot);
	}

	/***************************************************************************
	* Roll Pitch Yaw controllers, only run if enabled
	***************************************************************************/
	if (setpoint.en_r_ctrl) {
		// Roll
		mix_check_saturation(VEC_ROLL, mot, &min, &max);
		if (max > MAX_ROLL_COMPONENT)  max = MAX_ROLL_COMPONENT;
		if (min < -MAX_ROLL_COMPONENT) min = -MAX_ROLL_COMPONENT;
		//must fix an issue with [-0.0,+0.0] saturation:
		if (max < 0.01 * MAX_ROLL_COMPONENT && min > -0.01 * MAX_ROLL_COMPONENT) {
			//printf("WARNING: controller bounds are zeros, changing them to to MAX_ROLL_COMPONENT...\n");
			max = MAX_ROLL_COMPONENT;
			min = -MAX_ROLL_COMPONENT;
		}
		
		rc_filter_enable_saturation(&D_roll, min, max);
		D_roll.gain = D_roll_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_ROLL] = rc_filter_march(&D_roll, setpoint.roll - state_estimate.roll);
		mix_add_input(u[VEC_ROLL], VEC_ROLL, mot);
	}

	if (setpoint.en_py_ctrl) {
		// Pitch
		mix_check_saturation(VEC_PITCH, mot, &min, &max);
		if (max > MAX_PITCH_COMPONENT)  max = MAX_PITCH_COMPONENT;
		if (min < -MAX_PITCH_COMPONENT) min = -MAX_PITCH_COMPONENT;
		//printf("\n max = %f and min = %f \n", max, min);
		//must fix an issue with [-0.0,+0.0] saturation:
		if (max < 0.01 * MAX_PITCH_COMPONENT && min > -0.01 * MAX_PITCH_COMPONENT) {
			//printf("WARNING: controller bounds are zeros, changing them to to MAX_PITCH_COMPONENT...\n");
			max = MAX_PITCH_COMPONENT;
			min = -MAX_PITCH_COMPONENT;
		}
		rc_filter_enable_saturation(&D_pitch, min, max);
		D_pitch.gain = D_pitch_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_PITCH] = rc_filter_march(&D_pitch, -(setpoint.pitch - state_estimate.pitch)); //full PID control
		//u[VEC_PITCH] = (setpoint.pitch - state_estimate.pitch) * 2.0; //test using just a P controller
		mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
		
		//printf("\n setpoint.pitch = %f \n state_estimate.pitch = %f\n D_pitch.gain = %f\n u = %f\n ", setpoint.pitch, state_estimate.pitch, D_pitch.gain, u[VEC_PITCH]);

		// Yaw
		mix_check_saturation(VEC_YAW, mot, &min, &max);
		if (max > MAX_YAW_COMPONENT)  max = MAX_YAW_COMPONENT;
		if (min < -MAX_YAW_COMPONENT) min = -MAX_YAW_COMPONENT;
		//must fix an issue with [-0.0,+0.0] saturation:
		if (max < 0.01 * MAX_YAW_COMPONENT && min > -0.01 * MAX_YAW_COMPONENT) {
			//printf("WARNING: controller bounds are zeros, changing them to to MAX_YAW_COMPONENT...\n");
			max = MAX_YAW_COMPONENT;
			min = -MAX_YAW_COMPONENT;
		}
		rc_filter_enable_saturation(&D_yaw, min, max);
		D_yaw.gain = D_yaw_gain_orig * settings.v_nominal / state_estimate.v_batt_lp;
		u[VEC_YAW] = rc_filter_march(&D_yaw, -(setpoint.yaw - state_estimate.yaw));
		mix_add_input(u[VEC_YAW], VEC_YAW, mot);

		//printf("\n mot[0] = %f, mot[1] = %f, mot[2] = %f, mot[3] = %f \n", mot[0], mot[1], mot[2], mot[3]);
	}

	/***************************************************************************
	* Send Actuator signals immediately at the end of the control loop
	***************************************************************************/
	for (i = 0; i < settings.num_rotors; i++) {
		rc_saturate_double(&mot[i], 0.0, 1.0);
		fstate.m[i] = map_motor_signal(mot[i]);

		// final saturation just to take care of possible rounding errors
		// this should not change the values and is probably excessive
		rc_saturate_double(&fstate.m[i], 0.0, 1.0);

		// finally send mapped signal to servos:
        servos_march(i, &fstate.m[i]);
	}

	/***************************************************************************
	* Final cleanup, timing, and indexing
	***************************************************************************/
	// Load control inputs into cstate for viewing by outside threads
	for (i = 0; i < MAX_INPUTS; i++) fstate.u[i] = u[i];
	// keep track of loops since arming
	fstate.loop_index++;
	// log us since arming, mostly for the log
	fstate.last_step_ns = rc_nanos_since_boot();

	return 0;
}


int feedback_cleanup(void)
{
	//__send_motor_stop_pulse();

	servos_disarm();

	return 0;
}
