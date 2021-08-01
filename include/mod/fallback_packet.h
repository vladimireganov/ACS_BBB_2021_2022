#ifndef FALLBACK_PACKET_H
#define FALLBACK_PACKET_H

#include <setpoint_manager.h>

/*
<fallback_packet.h>

 * This header contains a packet structure with
 * everything essential for the system to perform well which can
 * be sent from supporting systems in the network

*/

/* Important!
* Altitude is always up / down, check with mix.c for coordinate frame definition
* Make sure all coordinate frames are properly matched! If using NED here, 
* make sure all of the data is properly converted all the data sent to this system.
*/
typedef struct fallback_packet_t
{
    uint32_t time;						///< Unique id or time for synchronizing transmition
	arm_state_t armed_state;			///< Use this channel to send ARMED or DISARMED command
    int run_preflight_checks;			///< 1 to start. Can only be used once (will not let you re-run the checklist to avoid issues during flight)
	int use_external_flight_state;		///< Use flight state determined externally (1)
	flight_status_t flight_state;		///< Use this channel to send current flight status
	int use_external_state_estimation;	///< Use data computed externally (1)
	double roll;						///< Rotation about x of the body 
	double pitch;						///< Rotation about y of the body 
	double yaw;							///< Rotation about z of the body 
	double proj_ap;						///< estimated projected apogee the vehicle will reach given current flight condition
	double alt;							///< altitude estimate
	double alt_vel;						///< vertical velocity estimate 
	double alt_accel;					///< vertical accel estimate 
} fallback_packet_t;

#define NUM_FRAMING_BYTES_SERIAL 4                 // 2 START bytes + 2 Fletcher-16 checksum bytes
#define SERIAL_DATA_LENGTH            sizeof(fallback_packet_t)      // Actual Packet Being Sent
#define SERIAL_PACKET_LENGTH	    SERIAL_DATA_LENGTH + NUM_FRAMING_BYTES_SERIAL

extern fallback_packet_t fallback;

#endif //FALLBACK