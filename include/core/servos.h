/**
 * @file servos.h
 *
 * @brief      Functions to send signals to the servos
 *
 * This is how the actuator command should be apllied to servos.
 * After initializing and arming was sucesfull, the feedback should call
 * servos_march to apply all of the servo signals using sstate.m[8] to 
 * active servos in sstate.ch[8]
 * 
 *
 */
#include <stdint.h> // for uint64_t
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <stdio.h>	// for printf
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/servo.h>
#include <rcs_defs.h>
#include <mix.h>
#include <tools.h>
#include <settings.h>
#include <rc/math/other.h>



#ifndef SERVOS_H
#define SERVOS_H

 /**
  * This is the state of the servo motors. Contains most recent values
  * reported by the servo motor function and should be only written to 
  * by the feedback controller and servo function itself after initialization.
  */
typedef struct servos_state_t {
	arm_state_t arm_state;			///< ARMED/DISARMED
	int initialized;				///< set to 1 after servos_init(void)
	double m[MAX_ROTORS];		///< servo motor signals for each pin in [0 1] range
	double m_us[MAX_ROTORS];	///< servo motor signals for each pin in pulse width
	double servos_lim[MAX_ROTORS][3];	///< servo minimum (first col.), nominal (second col.) and maximum values (last col.) 
}servos_state_t;


#ifndef test_servos
    #define num_cases 6
#endif  // !test_servos
typedef struct servos_preflight_test_t
{
    double time_delay;
    double time_delay_cases;
    uint64_t time_ns;
    uint64_t time_cases;
    uint64_t init_time;
    int pre_flight_check_res;
    int preflight_case;
    int init_cases;
    int initialized;
} servos_preflight_test_t;

extern servos_preflight_test_t servos_preflight;
extern servos_state_t sstate;

 /**
  * @brief      Initial setup of all servo motors. Should only be called
  *             once on program start.
  *
  * @return     0 on success, -1 on failure
  */
int servos_init(void);

/**
 * @brief      marches servos forward one step
 *
 * This is should be called at the end of feedback controller 
 * function to actually apply signals to servos.
 *
 * @return     0 on success, -1 on failure
 */
int servos_march(int i, double* mot);

/**
 * @brief      This is how outside functions should deactivate the servo motors.
 *
 *	Kills the power to the servo rail which makes any control ineffective.
 *	This should only be done when safe (on the ground).
 *				
 *
 * @return     0 on success, -1 on failure
 */
int servos_disarm(void);

/**
 * @brief      This is how outside functions should activate servo motors.
 *
 *	Provides power to servo rail, makes sure servo motors are ready
 *	to receive comands. Arming automatialy returns servos to their 
 *	nominal positions.
 * 
 * @return     0 on success, -1 on failure
 */
int servos_arm(void);

/**
 * @brief      This is how outside functions bring servos to nominal positions.
 *
 *	Keeps power to servo rail enabled (if disabled), makes sure servo motors 
 *  are receiving only nominal comands.
 *
 * @return     0 on success, -1 on failure
 */
int servos_return_to_nominal(void);

/**
* @brief		This function runs pre-flight check of low level logic
*					(control signal mapping, channel mixing and pwm)
*
*	@return		0 if still running, -1 on failure, 2 if completed
*/
int test_servos(void);


/**
 * @brief      Cleanup the servo motor function, freeing memory
 *
 * @return     0 on success, -1 on failure
 */
int servos_cleanup(void);






#endif // SERVOS_H