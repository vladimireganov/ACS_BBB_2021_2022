/**
 * @file servos.c
 */

#include <servos.h>

servos_state_t sstate;
servos_preflight_test_t servos_preflight;

/*
This defines operating range of each servo mottor.
First column should the minimum position of each servo 
corresponding to the servo pulse signal in miliseconds 
(see rc_test_servos.c). Second column is the nominal/safe 
value for servos to return to when armed and before being 
disarmed. Signal should be in the range of [500 2500]us.
For details, see rc_servo_send_pulse_us().
*/
static double servos_lim[8][3] = \
{ {1550.0, 1550.0, 1930.0}, \
{1525.0, 1540.0, 1960.0}, \
{1558.0, 1558.0, 1970.0}, \
{1548.0, 1548.0, 1940.0}, \
{1500.0, 1500.0, 1955.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}, \
{1500.0, 1500.0, 2000.0}};

/*
* This function should be used anytime 
* the servos need to be returned to 
* their nominal (safe) positions
*/
int __set_motor_nom_pulse(void)
{
    for (int i = 0; i < MAX_ROTORS; i++) {
        sstate.m_us[i] = servos_lim[i][1]; //have to set to calibrated nominal values
    }

    return 0;
}

/*
 * This function should be used anytime
 * all the servos need to be set to
 * their min positions
 */
int __set_motor_min_pulse(void)
{
    for (int i = 0; i < MAX_ROTORS; i++)
    {
        sstate.m_us[i] = servos_lim[i][0];  // have to set to calibrated min values
    }

    return 0;
}

/*
 * This function should be used anytime
 * all the servos need to be set to
 * their max positions
 */
int __set_motor_max_pulse(void)
{
    for (int i = 0; i < MAX_ROTORS; i++)
    {
        sstate.m_us[i] = servos_lim[i][2];  // have to set to calibrated max values
    }

    return 0;
}

/*
 * This function should be used anytime
 * all the servos need to be set to
 * their min positions
 */
int __set_single_min_max_pulse(int i, int pos)
{
    if (pos == 0) sstate.m_us[i] = servos_lim[i][0];  // have to set to calibrated min values
    else if (pos) sstate.m_us[i] = servos_lim[i][1];  // have to set to calibrated nom values
    else if (pos == 2)
        sstate.m_us[i] = servos_lim[i][2];  // have to set to calibrated max values
    else
    {
        printf("\nERROR: in __set_single_min_max_pulse, pos must be 0-min, 1-nom, or 2-max");
        return -1;
    }

    return 0;
}

double __map_servo_signal_ms(double* m, double servos_lim_min, double servos_lim_max)
{
    // sanity check
    if (*m > 1.0 || *m < 0.0) {
        printf("\nERROR: in __map_servo_signal_ms, desired thrust must be between 0.0 & 1.0");
        return -1;
    }

    // return quickly for boundary conditions
    if (*m == 0.0) return servos_lim_min;
    if (*m == 1.0) return servos_lim_max;

    // Map [0 1] signal to servo pulse width
    return *m * (servos_lim_max - servos_lim_min) + servos_lim_min;

    fprintf(stderr, "ERROR: something in __map_servo_signal_ms went wrong\n");
    return -1;
}

int servos_init(void)
{
    sstate.arm_state = DISARMED;
    // initialize PRU
    if (rc_servo_init()) return -1;

    for (int i = 0; i < MAX_ROTORS; i++) {
        sstate.m[i] = 0; //zero everything out just in case
    }

    servos_preflight.initialized = 0;
    servos_preflight.preflight_case = 0;
    //printf("\nInitializing servos...\n");

    __set_motor_nom_pulse();

    sstate.initialized = 1;
    return 0;
}

int servos_arm(void)
{
    if (sstate.arm_state == ARMED) {
        printf("WARNING: trying to arm when servos are already armed\n");
        return 0;
    }
    if (sstate.initialized != 1)
    {
        printf("Servos have not been initialized \n");
        return -1;
    }
    // need to set each of the servos to their nominal positions:
    __set_motor_nom_pulse();


    //enable power:
    rc_servo_power_rail_en(1);

    sstate.arm_state = ARMED; //set servos to armed and powered
    return 0;
}

int servos_return_to_nominal(void)
{
    if (sstate.initialized != 1)
    {
        printf("Servos have not been initialized \n");
        return -1;
    }

    __set_motor_nom_pulse(); //do this every time to ensure nominal position

    if (sstate.arm_state == DISARMED) {
        //no need to proceed if already disarmed (no power to servo rail)
        return 0;
    }

    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < MAX_ROTORS; i++) {
        if (rc_servo_send_pulse_us(i, sstate.m_us[i]) == -1) return -1;
    }
    return 0;
}

int servos_disarm(void)
{
    // need to set each of the servos to their nominal positions:
    __set_motor_nom_pulse(); //won't work, need extra time before power is killed

    //send servo signals using Pulse Width in microseconds
    for (int i = 0; i < MAX_ROTORS; i++) {
        if (rc_servo_send_pulse_us(i, sstate.m_us[i]) == -1) return -1;
    }

    //power-off servo rail:
    rc_servo_power_rail_en(0);

    sstate.arm_state = DISARMED;
    return 0;
}

int servos_march(int i, double* mot)
{
    if (sstate.arm_state == DISARMED) {
        //printf("WARNING: trying to march servos when servos disarmed\n");
        return 0;
    }

    // need to do mapping between [0 1] and servo signal in us
    sstate.m_us[i] = __map_servo_signal_ms(mot, servos_lim[i][0], servos_lim[i][2]);

    //send servo signals using [-1.5 1.5] normalized values
    //if (rc_servo_send_pulse_normalized(i, mot) == -1) return -1;

    //send servo signals using Pulse Width in microseconds
    if (rc_servo_send_pulse_us(i+1, sstate.m_us[i]) == -1) return -1;

    return 0;
}

int test_servos(void)
{
    if (servos_preflight.initialized && servos_preflight.pre_flight_check_res) return 0; //run only once to prevent errors
    double u[MAX_INPUTS], mot[MAX_ROTORS];
    int i;

    if (servos_preflight.preflight_case == 0 && finddt_s(servos_preflight.init_time) < 5.0)
        return 0;

    for (i = 0; i < MAX_ROTORS; i++) mot[i] = 0.0;
    for (i = 0; i < MAX_INPUTS; i++) u[i] = 0.0;

    //run only once
    if (servos_preflight.preflight_case == 0)
    {
        printf("Initializing pre-fligth checks:\n");
        // Start by zeroing out the motors signals and then add from there.
        servos_preflight.preflight_case = 1;
        user_input.requested_arm_mode = ARMED;
        servos_preflight.init_time = rc_nanos_since_boot();
        servos_preflight.pre_flight_check_res = 0;
        servos_preflight.init_cases = 1;
        servos_preflight.initialized = 1;
    }

    //1. test min/max pulses
    if (servos_preflight.preflight_case == 1)
    {
        if (servos_preflight.init_cases == 1)
        {
            servos_preflight.init_cases = 2;
            servos_preflight.time_delay = 5.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            printf("Case-1: min/max pulses check\n");
            if (__set_motor_max_pulse()) printf("ERROR: Failed to send the maximum pulse.\n");
        }
        else if (servos_preflight.init_cases == 2 &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay)
        {
            if (__set_motor_max_pulse()) printf("ERROR: Failed to send the maximum pulse.\n");
        }
        else if (servos_preflight.init_cases == 2 &&
                 finddt_s(servos_preflight.time_ns) >= servos_preflight.time_delay)
        {

            if (__set_motor_min_pulse()) printf("ERROR: Failed to send the minimum pulse.\n");
            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 1.0;
            servos_preflight.preflight_case = 2;
            printf("Case-1: Done\n");
        }

        for (i = 0; i < settings.num_rotors; i++)
            if (rc_servo_send_pulse_us(i + 1, sstate.m_us[i]) == -1)
                printf("ERROR: Failed to send pulse to servo rail pin %d\n", i + 1);
        return 0;
    }
    // 2. test min/max signal mapping
    else if (servos_preflight.preflight_case == 2)
    {
        if (servos_preflight.init_cases == 2 && finddt_s(servos_preflight.time_cases) >= servos_preflight.time_delay_cases)
        {
            servos_preflight.init_cases = 3;
            servos_preflight.time_delay = 5.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            printf("Case-2: min/max signal mapping\n");
            for (i = 0; i < settings.num_rotors; i++) mot[i] = 1.0;
        }
        else if (servos_preflight.init_cases == 3 &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay)
        {
            for (i = 0; i < settings.num_rotors; i++) mot[i] = 1.0;
        }
        else if (servos_preflight.init_cases == 3 && finddt_s(servos_preflight.time_ns) >=
                 servos_preflight.time_delay)
        {
            for (i = 0; i < settings.num_rotors; i++) mot[i] = 0.0;
            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 1.0;
            servos_preflight.preflight_case = 3;
            printf("Case-2: Done\n");
        }

        for (i = 0; i < settings.num_rotors; i++)
        {
            fstate.m[i] = map_motor_signal(mot[i]);

            rc_saturate_double(&fstate.m[i], 0.0, 1.0);
            // finally send mapped signal to servos:
            servos_march(i, &fstate.m[i]);
        }
        return 0;
    }
    // 3. test min/max pitch channel mixing
    else if (servos_preflight.preflight_case == 3)
    {
        if (servos_preflight.init_cases == 3 && finddt_s(servos_preflight.time_cases) >= servos_preflight.time_delay_cases)
        {
            servos_preflight.init_cases = 4;
            servos_preflight.time_delay = 3.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            printf("Case-3: min/max pitch channel mixing\n");
            u[VEC_PITCH] = 1.0;
            mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
        }
        else if (servos_preflight.init_cases == 4 &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay)
        {
            u[VEC_PITCH] = 1.0;
            mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
        }
        else if (servos_preflight.init_cases == 4 &&
                 finddt_s(servos_preflight.time_ns) >=
                     servos_preflight.time_delay &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay*2.0)
        {
            u[VEC_PITCH] = -1.0;
            mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
        }
        else if (servos_preflight.init_cases == 4 &&
                 finddt_s(servos_preflight.time_ns) >= servos_preflight.time_delay*2.0)
        {
            u[VEC_PITCH] = 0.0;
            mix_add_input(u[VEC_PITCH], VEC_PITCH, mot);
            servos_preflight.preflight_case = 4;
            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 0.5;
            printf("Case-3: Done\n");
        }

        for (i = 0; i < settings.num_rotors; i++)
        {

            rc_saturate_double(&mot[i], 0.0, 1.0);
            fstate.m[i] = map_motor_signal(mot[i]);

            rc_saturate_double(&fstate.m[i], 0.0, 1.0);
            // finally send mapped signal to servos:
            servos_march(i, &fstate.m[i]);
        }
        return 0;
    }
    // 4. test min/max yaw channel mixing
    else if (servos_preflight.preflight_case == 4)
    {
        if (servos_preflight.init_cases == 4 && finddt_s(servos_preflight.time_cases) >= servos_preflight.time_delay_cases)
        {
            servos_preflight.init_cases = 5;
            servos_preflight.time_delay = 3.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            printf("Case-4: min/max yaw channel mixing\n");
            u[VEC_YAW] = 1.0;
            mix_add_input(u[VEC_YAW], VEC_YAW, mot);
        }
        else if (servos_preflight.init_cases == 5 &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay)
        {
            u[VEC_YAW] = 1.0;
            mix_add_input(u[VEC_YAW], VEC_YAW, mot);
        }
        else if (servos_preflight.init_cases == 5 &&
                 finddt_s(servos_preflight.time_ns) >= servos_preflight.time_delay &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay * 2.0)
        {
            u[VEC_YAW] = -1.0;
            mix_add_input(u[VEC_YAW], VEC_YAW, mot);
        }
        else if (servos_preflight.init_cases == 5 &&
                 finddt_s(servos_preflight.time_ns) >= servos_preflight.time_delay * 2.0)
        {
            u[VEC_YAW] = 0.0;
            mix_add_input(u[VEC_YAW], VEC_YAW, mot);

            servos_preflight.preflight_case = 5;
            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 1.0;
            printf("Case-4: Done\n");
        }

        for (i = 0; i < settings.num_rotors; i++)
        {

            rc_saturate_double(&mot[i], 0.0, 1.0);
            fstate.m[i] = map_motor_signal(mot[i]);

            rc_saturate_double(&fstate.m[i], 0.0, 1.0);
            // finally send mapped signal to servos:
            servos_march(i, &fstate.m[i]);
        }
        return 0;
    }
    // 5. test min/max brake channel mixing
    else if (servos_preflight.preflight_case == 5)
    {
        if (servos_preflight.init_cases == 5 &&
            finddt_s(servos_preflight.time_cases) >= servos_preflight.time_delay_cases)
        {
            servos_preflight.init_cases = 6;
            servos_preflight.time_delay = 3.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            u[VEC_X] = -1.0;
            printf("Case-4: min/max brake channel mixing\n");
            mix_add_input(u[VEC_X], VEC_X, mot);
        }
        else if (servos_preflight.init_cases == 6 &&
                 finddt_s(servos_preflight.time_ns) < servos_preflight.time_delay)
        {
            u[VEC_X] = -1.0;
            mix_add_input(u[VEC_X], VEC_X, mot);
        }
        else if (servos_preflight.init_cases == 6 &&
                 finddt_s(servos_preflight.time_ns) >= servos_preflight.time_delay)
        {
            u[VEC_X] = 0.0;
            mix_add_input(u[VEC_X], VEC_X, mot);

            servos_preflight.preflight_case = 6;
            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 0.5;
            printf("Case-5: Done\n");
        }

        for (i = 0; i < settings.num_rotors; i++)
        {
            rc_saturate_double(&mot[i], 0.0, 1.0);
            fstate.m[i] = map_motor_signal(mot[i]);

            rc_saturate_double(&fstate.m[i], 0.0, 1.0);
            // finally send mapped signal to servos:
            servos_march(i, &fstate.m[i]);
        }
        return 0;
    }
    // 6. test incremental increase from min to max on brake channel
    else if (servos_preflight.preflight_case == 6)
    {
        if (servos_preflight.init_cases == 6 && finddt_s(servos_preflight.time_cases) >= servos_preflight.time_delay_cases)
        {
            servos_preflight.init_cases = 7;
            servos_preflight.time_delay = 3.0;
            servos_preflight.time_ns = rc_nanos_since_boot();

            u[VEC_X] = 0.0;
            printf("Case-6: incremental increase from min to max on brake channel\n");
            mix_add_input(u[VEC_X], VEC_X, mot);
        }
        else if (servos_preflight.init_cases == 7 &&
                 finddt_s(servos_preflight.time_ns) <= servos_preflight.time_delay)
        {
            u[VEC_X] = -finddt_s(servos_preflight.time_ns) / servos_preflight.time_delay;
            mix_add_input(u[VEC_X], VEC_X, mot);

            servos_preflight.time_cases = rc_nanos_since_boot();
            servos_preflight.time_delay_cases = 1.0;
        }
        else if (servos_preflight.init_cases == 7 &&
                 finddt_s(servos_preflight.time_ns) > servos_preflight.time_delay &&
                 finddt_s(servos_preflight.time_ns) <= servos_preflight.time_delay + 1.0)
        {
            __set_motor_nom_pulse();
        }
        else if (servos_preflight.init_cases == 7 &&
                 finddt_s(servos_preflight.time_ns) > servos_preflight.time_delay + 1.0)
        {
            printf("Servo Test Completed\n");
            user_input.requested_arm_mode = DISARMED;
            servos_preflight.preflight_case = 7;
            servos_preflight.pre_flight_check_res = 1;
            return 2;
        }

        for (i = 0; i < settings.num_rotors; i++)
        {
            rc_saturate_double(&mot[i], 0.0, 1.0);
            fstate.m[i] = map_motor_signal(mot[i]);

            rc_saturate_double(&fstate.m[i], 0.0, 1.0);
            // finally send mapped signal to servos:
            servos_march(i, &fstate.m[i]);
        }
        return 0;
    }
    else 
    {
        printf("ERROR: Unknown case\n");
        return -1;
    }
    return 0;
}



int servos_cleanup(void)
{
    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();
    return 0;
}