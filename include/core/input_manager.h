/**
 * <input_manager.h>
 *
 * Function to manage external inputs, ARMING and DISARMING;
 * 
 * TODO: allow use of external state estimation parameters
 */
#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H
#include <flight_mode.h>
#include <feedback.h> // only for arm_state_t
#include <stdbool.h>

/**
 * Represents current command by the user. This is populated by the
 * input_manager thread which decides to read from mavlink or DSM depending on
 * what it is receiving.
 */
typedef struct user_input_t{
	int initialized; 					///< set to 1 after input_manager_init(void)
	flight_mode_t flight_mode;			///< this is the user commanded flight_mode.
	int input_active;					///< nonzero indicates some user control is coming in
	arm_state_t requested_arm_mode;		///< set to ARMED after arming sequence is entered.
	int use_external_state_estimation;	///< 1 if we want to use externally computed values
	int use_external_flight_state;		///< 1 if we want to use externally determined flight states.
    int run_preflight_checks;			///< 1 to start. Can only be used once (will not let you re-run the checklist to avoid issues during flight)
} user_input_t;

extern user_input_t user_input;

/**
 * @brief      Starts an input manager thread.
 *
 *             Watch for new DSM data and translate into local user mode. Used
 *             in input_manager.c
 *
 * @return     0 on success, -1 on failure
 */
int input_manager_init(void);

/**
 * @brief      Picks source of state estimation.
 *
 *             
 *
 * @return     0 on success, -1 on failure
 */
int pick_data_source(void);

/**
 * @brief      Starts the pre-flight check algorithm.
 *
 *
 *
 * @return     1 on success, -1 on failure, 0 while active
 */
int start_pre_flight_checks(void);

/**
 * @brief      Waits for the input manager thread to exit
 *
 *             This should only be called after the program flow state is set to
 *             EXITING as that's the only thing that will cause the thread to
 *             exit on its own safely. Used in input_manager.c
 *
 * @return     0 on clean exit, -1 if exit timed out.
 */
int input_manager_cleanup(void);

#endif // INPUT_MANAGER_H
