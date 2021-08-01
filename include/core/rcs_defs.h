/**
 * <rcs_defs.h>
 *
 * @brief constants and parameters
 */

#ifndef RCS_DEFS_H
#define RCS_DEFS_H

 /**
  * @brief      ARMED or DISARMED to indicate if the feedback controller is
  *             allowed to output to the motors
  */
typedef enum arm_state_t {
	DISARMED,
	ARMED
} arm_state_t;

// Speed of feedback loop
#define FEEDBACK_HZ		200
#define DT			0.005

//IMU Parameters
#define IMU_PRIORITY    51
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// math constants
#define GRAVITY			9.80665	///< one G m/s^2

// order of control inputs
// brake(X), roll, pitch, yaw
#define VEC_X			0	//central axis, in the direction of nominal flight
#define VEC_Y			1	//
#define VEC_Z			2	// 
#define VEC_ROLL		3   //rotation about X
#define VEC_PITCH		4	//rotation about Y
#define VEC_YAW			5	//rotation about Z

// user control parameters
#define MAX_YAW_RATE		2.5	// rad/s
#define MAX_ROLL_SETPOINT	0.2	// rad
#define MAX_PITCH_SETPOINT	0.2	// rad
#define ALT_MAX_ERROR		300.0	//< meters.
#define YAW_DEADZONE		0.02
#define SOFT_START_SECONDS	1.0	// controller soft start seconds
#define ALT_CUTOFF_FREQ		2.0
#define BMP_RATE_DIV		10	// optionally sample bmp less frequently than mpu

// controller absolute limits
#define MAX_ROLL_COMPONENT	1.0
#define MAX_PITCH_COMPONENT	1.0
#define MAX_YAW_COMPONENT	1.0

#define MAX_X_COMPONENT		1.0
#define TIP_ANGLE			1.13

 // Files
//#define LOG_DIR		"/home/debian/rcs_logs/"
#define LOG_DIR		"/mnt/SD/rcs_logs/"

// terminal emulator control sequences
#define WRAP_DISABLE	"\033[?7l"
#define WRAP_ENABLE	"\033[?7h"
#define KNRM		"\x1B[0m"	// "normal" to return to default after colour
#define KRED		"\x1B[31m"
#define KGRN		"\x1B[32m"
#define KYEL		"\x1B[33m"
#define KBLU		"\x1B[34m"
#define KMAG		"\x1B[35m"
#define KCYN		"\x1B[36m"
#define KWHT		"\x1B[37m"

//#define DEBUG

#endif // RCS_DEFS_H
