/**
 * <fly/settings.h>
 *
 * @brief      Functions to read the json settings file
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <rc/math/filter.h>
#include <rc/mpu.h>

#include <flight_mode.h>
#include <thrust_map.h>
#include <mix.h>
#include <input_manager.h>
#include <rcs_defs.h>

 /**
  * @brief enum for possible vertial axis defined here
  *
  * possible orientations for vertial axis, used in state_estimator.c
  
typedef enum vertical_axis_t {
	ORIENTATION_X_UP,
	ORIENTATION_Z_DOWN,
} vertical_axis_t;
 */

/**
 * Configuration settings read from the json settings file and passed to most
 * threads as they initialize.
 */
typedef struct settings_t{
	/** @name File details */
	char name[128]; ///< string declaring the name of the settings file
	///@}

	/**@name warings */
	///@{
	int warnings_en;
	///@}

	/** @name physical parameters */
	///@{
	int num_rotors;
	rotor_layout_t layout;
	int dof;
	thrust_map_t thrust_map;
	rc_mpu_orientation_t orientation;
	double v_nominal;
	double v_nominal_jack;
	double target_altitude_m;
	double event_launch_accel;
	double event_launch_dh;
	double event_ignition_dh;
	double event_ignition_delay_s;
	double event_cutoff_delay_s;
	double event_cutoff_dh;
	double event_apogee_delay_s;
	double event_apogee_accel_tol;
	double event_apogee_dh;
	double event_landing_delay_early_s;
	double event_landing_delay_late_s;
	double event_start_landing_alt_m;
	double event_landing_alt_tol;
	double event_landing_vel_tol;
	double event_landning_accel_tol;
	int enable_magnetometer; // we suggest leaving as 0 (mag OFF)
	int enable_xbee;	//enable xbee serial link
	int use_xbee_yaw;
	int use_xbee_pitch;
	int use_xbee_roll;
	int enable_encoders;
	int enable_serial;
    int enable_send_serial;
    int enable_receive_serial;
    double serial_send_update_hz;
	char serial_port_1[20];
	char serial_port_2[20];
	int serial_port_1_baud;
	int serial_port_2_baud;
	///@}

	/** @name printf settings */
	///@{
	int printf_arm;
	int printf_battery;
	int printf_altitude;
	int printf_proj_ap;
	int printf_rpy;
	int printf_setpoint;
	int printf_u;
	int printf_motors;
	int printf_mode;
	int printf_status;
	int printf_xbee;
	int printf_rev;
	int printf_counter;
	///@}

	/** @name log settings */
	///@{
	int enable_logging;
	int log_sensors;
	int log_state;
	int log_setpoint;
	int log_control_u;
	int log_motor_signals;
    int log_motor_signals_us;
	int log_encoders;
	///@}

	/** @name mavlink stuff */
	///@{
	char dest_ip[24];
	uint8_t my_sys_id;
	uint16_t mav_port;

	/** @name feedback controllers */
	///@{
	rc_filter_t roll_controller;
	rc_filter_t pitch_controller;
	rc_filter_t yaw_controller;
	rc_filter_t altitude_controller;
	///@}

}settings_t;

/**
 * settings are external, so just include this header and read from it
 */
extern settings_t settings;

/**
 * @brief      Populates the settings and controller structs with the settings file.
 *
 * @return     0 on success, -1 on failure
 */
int settings_load_from_file(char* path);


/**
 * @brief      Only used in debug mode. Prints settings to console
 *
 * @return     0 on success, -1 on failure
 */
int settings_print(void);



#endif // SETTINGS_H
