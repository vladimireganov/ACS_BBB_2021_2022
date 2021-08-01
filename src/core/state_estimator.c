/**
 * @file state_estimator.c
 *
 */

#include <stdio.h>
#include <math.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/quaternion.h>
#include <rc/math/matrix.h>
#include <rc/math/other.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>

#include <rcs_defs.h>
#include <state_estimator.h>
#include <settings.h>
#include <xbee_packet_t.h>
#include <setpoint_manager.h>
#include <fallback_packet.h>

#define TWO_PI (M_PI*2.0)

state_estimate_t state_estimate; // extern variable in state_estimator.h
//fallback_packet_t main_state; //extern in fallback_packet.h

// sensor data structs
rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;

// battery filter
static rc_filter_t batt_lp		= RC_FILTER_INITIALIZER;
static rc_filter_t batt_lp_jack = RC_FILTER_INITIALIZER;

// altitude filter components
static rc_kalman_t alt_kf = RC_KALMAN_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;

// This function estimates the apogee using current state of the vehicle
static void __projected_altitude(void) {
	state_estimate.proj_ap = fabs(state_estimate.alt_bmp_vel * 
		state_estimate.alt_bmp_vel) / 
		(2.0 * fabs(state_estimate.alt_bmp_accel + GRAVITY)) * 
		log(fabs((state_estimate.alt_bmp_accel) / GRAVITY)) +
		state_estimate.alt_bmp;
	return;
}


static void __batt_init(void)
{
	// init the battery low pass filter
	rc_filter_moving_average(&batt_lp_jack, 20, DT);
	double dc_read_jack = rc_adc_dc_jack();
	if (dc_read_jack < 3.0){
		if (settings.warnings_en) {
			fprintf(stderr, "WARNING: ADC read %0.1fV on the barrel jack. Please connect\n", dc_read_jack);
			fprintf(stderr, "battery to barrel jack, assuming nominal voltage for now.\n");
		}
		dc_read_jack = settings.v_nominal_jack;
	}
	rc_filter_prefill_inputs(&batt_lp_jack, dc_read_jack);
	rc_filter_prefill_outputs(&batt_lp_jack, dc_read_jack);


	// init the battery low pass filter
	rc_filter_moving_average(&batt_lp, 20, DT);
	double dc_read = rc_adc_dc_jack();
	if (dc_read < 3.0) {
		if (settings.warnings_en) {
			fprintf(stderr, "WARNING: ADC read %0.1fV on the 2S lipo connector. Please connect\n", dc_read);
			fprintf(stderr, "battery to 2S connector, assuming nominal voltage for now.\n");
		}
		dc_read = settings.v_nominal;
	}
	rc_filter_prefill_inputs(&batt_lp, dc_read);
	rc_filter_prefill_outputs(&batt_lp, dc_read);
	return;
	return;
}



static void __batt_march(void)
{
	double tmp		= rc_adc_batt();
	if(tmp<3.0) tmp	= settings.v_nominal;

	state_estimate.v_batt_raw = tmp;
	state_estimate.v_batt_lp = rc_filter_march(&batt_lp, tmp);


	double tmp_jack = rc_adc_dc_jack();
	if (tmp_jack < 3.0) tmp_jack = settings.v_nominal_jack;

	state_estimate.v_batt_raw_jack = tmp_jack;
	state_estimate.v_batt_lp_jack = rc_filter_march(&batt_lp_jack, tmp_jack);
	return;
}

static void __batt_cleanup(void)
{
	rc_filter_free(&batt_lp);
	rc_filter_free(&batt_lp_jack);
	return;
}



static void __imu_march(void)
{
	static double last_roll		= 0.0;
	static int num_roll_spins	= 0;
	static double last_yaw		= 0.0;
	static int num_yaw_spins	= 0;
	double diff;
	if (settings.enable_xbee) {
		state_estimate.quat_mocap[0] = xbeeMsg.qw; // W
		state_estimate.quat_mocap[1] = xbeeMsg.qx; // X (i)
		state_estimate.quat_mocap[2] = xbeeMsg.qy; // Y (j)
		state_estimate.quat_mocap[3] = xbeeMsg.qz; // Z (k)

		// normalize quaternion because we don't trust the mocap system
		rc_quaternion_norm_array(state_estimate.quat_mocap);
		// calculate tait bryan angles too
		rc_quaternion_to_tb_array(state_estimate.quat_mocap, state_estimate.tb_mocap);
		// position
		state_estimate.pos_mocap[0] = (double)xbeeMsg.x;
		state_estimate.pos_mocap[1] = (double)xbeeMsg.y;
		state_estimate.pos_mocap[2] = (double)xbeeMsg.z;
	}

	switch (settings.orientation)
	{
	case ORIENTATION_X_UP:
		// gyro and accel require converting to body frame
		state_estimate.gyro[0] = mpu_data.gyro[2];
		state_estimate.gyro[1] = mpu_data.gyro[1];
		state_estimate.gyro[2] = -mpu_data.gyro[0];
		state_estimate.accel[0] = mpu_data.accel[2];
		state_estimate.accel[1] = mpu_data.accel[1];
		state_estimate.accel[2] = -mpu_data.accel[0];

		// quaternion also needs coordinate transform
		state_estimate.quat_imu[0] = mpu_data.dmp_quat[0]; // W
		state_estimate.quat_imu[1] = mpu_data.dmp_quat[3]; // X (i)
		state_estimate.quat_imu[2] = mpu_data.dmp_quat[2]; // Y (j)
		state_estimate.quat_imu[3] = -mpu_data.dmp_quat[1]; // Z (k)

		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_imu);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

		// roll is more annoying since we have to detect spins
		// also make sign negative since NED coordinates has Z point down
		diff = state_estimate.tb_imu[2] + (num_roll_spins * TWO_PI) - last_roll;
		//detect the crossover point at +-PI and update num yaw spins
		if (diff < -M_PI) num_roll_spins++;
		else if (diff > M_PI) num_roll_spins--;

		// finally the new value can be written
		state_estimate.imu_continuous_roll = state_estimate.tb_imu[0] + (num_roll_spins * TWO_PI);
		last_roll = state_estimate.imu_continuous_roll;
		return;
	case ORIENTATION_Z_DOWN:
		// gyro and accel require converting to NED coordinates
		state_estimate.gyro[0] = mpu_data.gyro[1];
		state_estimate.gyro[1] = mpu_data.gyro[0];
		state_estimate.gyro[2] = -mpu_data.gyro[2];
		state_estimate.accel[0] = mpu_data.accel[1];
		state_estimate.accel[1] = mpu_data.accel[0];
		state_estimate.accel[2] = -mpu_data.accel[2];

		// quaternion also needs coordinate transform
		state_estimate.quat_imu[0] = mpu_data.dmp_quat[0]; // W
		state_estimate.quat_imu[1] = mpu_data.dmp_quat[2]; // X (i)
		state_estimate.quat_imu[2] = mpu_data.dmp_quat[1]; // Y (j)
		state_estimate.quat_imu[3] = -mpu_data.dmp_quat[3]; // Z (k)

		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_imu);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

		// yaw is more annoying since we have to detect spins
		// also make sign negative since NED coordinates has Z point down
		diff = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI) - last_yaw;
		//detect the crossover point at +-PI and update num yaw spins
		if (diff < -M_PI) num_yaw_spins++;
		else if (diff > M_PI) num_yaw_spins--;

		// finally the new value can be written
		state_estimate.imu_continuous_yaw = state_estimate.tb_imu[2] + (num_yaw_spins * TWO_PI);
		last_yaw = state_estimate.imu_continuous_yaw;
		return;
	default:
		fprintf(stderr, "ERROR: Unknown Body Frame Orientation. Assuming ORIENTATION_X_UP. Please check settings.orientation\n");

		// gyro and accel require converting to body frame
		state_estimate.gyro[0] = mpu_data.gyro[2];
		state_estimate.gyro[1] = mpu_data.gyro[1];
		state_estimate.gyro[2] = -mpu_data.gyro[0];
		state_estimate.accel[0] = mpu_data.accel[2];
		state_estimate.accel[1] = mpu_data.accel[1];
		state_estimate.accel[2] = -mpu_data.accel[0];

		// quaternion also needs coordinate transform
		state_estimate.quat_imu[0] = mpu_data.dmp_quat[0]; // W
		state_estimate.quat_imu[1] = mpu_data.dmp_quat[3]; // X (i)
		state_estimate.quat_imu[2] = mpu_data.dmp_quat[2]; // Y (j)
		state_estimate.quat_imu[3] = -mpu_data.dmp_quat[1]; // Z (k)

		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_imu);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_imu, state_estimate.tb_imu);

		// roll is more annoying since we have to detect spins
		// also make sign negative since NED coordinates has Z point down
		diff = state_estimate.tb_imu[2] + (num_roll_spins * TWO_PI) - last_roll;
		//detect the crossover point at +-PI and update num yaw spins
		if (diff < -M_PI) num_roll_spins++;
		else if (diff > M_PI) num_roll_spins--;

		// finally the new value can be written
		state_estimate.imu_continuous_roll = state_estimate.tb_imu[0] + (num_roll_spins * TWO_PI);
		last_roll = state_estimate.imu_continuous_roll;
		return;
	}
}


static void __mag_march(void)
{
	static double last_roll		= 0.0;
	static int num_roll_spins	= 0;
	static double last_yaw		= 0.0;
	static int num_yaw_spins	= 0;
	double diff = 0;
	switch (settings.orientation)
	{
	case ORIENTATION_X_UP:
		// don't do anything if mag isn't enabled
		if (!settings.enable_magnetometer) return;

		// mag require converting to body coordinates
		state_estimate.mag[0] = mpu_data.mag[2];
		state_estimate.mag[1] = mpu_data.mag[1];
		state_estimate.mag[2] = -mpu_data.mag[0];

		// quaternion also needs coordinate transform
		state_estimate.quat_mag[0] = mpu_data.fused_quat[0]; // W
		state_estimate.quat_mag[1] = mpu_data.fused_quat[3]; // X (i)
		state_estimate.quat_mag[2] = mpu_data.fused_quat[2]; // Y (j)
		state_estimate.quat_mag[3] = -mpu_data.fused_quat[1]; // Z (k)


		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_mag);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

		// heading
		state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
		state_estimate.mag_heading = state_estimate.tb_mag[0];

		// roll is more annoying since we have to detect spins
		diff = state_estimate.tb_mag[0] + (num_roll_spins * TWO_PI) + last_roll;
		//detect the crossover point at +-PI and update num roll spins
		if (diff < -M_PI) num_roll_spins++;
		else if (diff > M_PI) num_roll_spins--;

		// finally the new value can be written
		state_estimate.mag_heading_continuous = state_estimate.tb_mag[0] + (num_roll_spins * TWO_PI);
		break;
	case ORIENTATION_Z_DOWN:
		// don't do anything if mag isn't enabled
		if (!settings.enable_magnetometer) return;

		// mag require converting to NED coordinates
		state_estimate.mag[0] = mpu_data.mag[1];
		state_estimate.mag[1] = mpu_data.mag[0];
		state_estimate.mag[2] = -mpu_data.mag[2];

		// quaternion also needs coordinate transform
		state_estimate.quat_mag[0] = mpu_data.fused_quat[0]; // W
		state_estimate.quat_mag[1] = mpu_data.fused_quat[2]; // X (i)
		state_estimate.quat_mag[2] = mpu_data.fused_quat[1]; // Y (j)
		state_estimate.quat_mag[3] = -mpu_data.fused_quat[3]; // Z (k)


		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_mag);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

		// heading
		state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
		state_estimate.mag_heading = state_estimate.tb_mag[2];

		// yaw is more annoying since we have to detect spins
		// also make sign negative since NED coordinates has Z point down
		diff = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI) - last_yaw;
		//detect the crossover point at +-PI and update num yaw spins
		if (diff < -M_PI) num_yaw_spins++;
		else if (diff > M_PI) num_yaw_spins--;

		// finally the new value can be written
		state_estimate.mag_heading_continuous = state_estimate.tb_mag[2] + (num_yaw_spins * TWO_PI);
		break;
	default:
		fprintf(stderr, "ERROR: Unknown Body Frame Orientation. Assuming ORIENTATION_X_UP. Please check settings.orientation\n");

		// don't do anything if mag isn't enabled
		if (!settings.enable_magnetometer) return;

		// mag require converting to body coordinates
		state_estimate.mag[0] = mpu_data.mag[2];
		state_estimate.mag[1] = mpu_data.mag[1];
		state_estimate.mag[2] = -mpu_data.mag[0];

		// quaternion also needs coordinate transform
		state_estimate.quat_mag[0] = mpu_data.fused_quat[0]; // W
		state_estimate.quat_mag[1] = mpu_data.fused_quat[3]; // X (i)
		state_estimate.quat_mag[2] = mpu_data.fused_quat[2]; // Y (j)
		state_estimate.quat_mag[3] = -mpu_data.fused_quat[1]; // Z (k)


		// normalize it just in case
		rc_quaternion_norm_array(state_estimate.quat_mag);
		// generate tait bryan angles
		rc_quaternion_to_tb_array(state_estimate.quat_mag, state_estimate.tb_mag);

		// heading
		state_estimate.mag_heading_raw = mpu_data.compass_heading_raw;
		state_estimate.mag_heading = state_estimate.tb_mag[0];

		// roll is more annoying since we have to detect spins
		diff = state_estimate.tb_mag[0] + (num_roll_spins * TWO_PI) + last_roll;
		//detect the crossover point at +-PI and update num roll spins
		if (diff < -M_PI) num_roll_spins++;
		else if (diff > M_PI) num_roll_spins--;

		// finally the new value can be written
		state_estimate.mag_heading_continuous = state_estimate.tb_mag[0] + (num_roll_spins * TWO_PI);
	}
	
	last_roll = state_estimate.mag_heading_continuous;
	return;
}

/**
 * @brief      initialize the altitude kalman filter
 *
 * @return     0 on success, -1 on failure
 */
static int __altitude_init(void)
{

	//initialize altitude kalman filter and bmp sensor
	rc_matrix_t F = RC_MATRIX_INITIALIZER;
	rc_matrix_t G = RC_MATRIX_INITIALIZER;
	rc_matrix_t H = RC_MATRIX_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

	const int Nx = 3;
	const int Ny = 1;
	const int Nu = 1;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5*DT*DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if(rc_kalman_alloc_lin(&alt_kf,F,G,H,Q,R,Pi)==-1) return -1;
	rc_matrix_free(&F);
	rc_matrix_free(&G);
	rc_matrix_free(&H);
	rc_matrix_free(&Q);
	rc_matrix_free(&R);
	rc_matrix_free(&Pi);

	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, DT, 20*DT)) return -1;

	// init barometer and read in first data
	if(rc_bmp_read(&bmp_data)) return -1;

	return 0;
}

static void __altitude_march(void)
{
	int i;
	double accel_vec[3];
	static rc_vector_t u = RC_VECTOR_INITIALIZER;
	static rc_vector_t y = RC_VECTOR_INITIALIZER;

	// grab raw data
	state_estimate.bmp_pressure_raw = bmp_data.pressure_pa;
	state_estimate.alt_bmp_raw		= bmp_data.alt_m;
	state_estimate.bmp_temp			= bmp_data.temp_c;



	// make copy of acceleration reading before rotating
	for(i=0;i<3;i++) accel_vec[i] = state_estimate.accel[i];

	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec, state_estimate.quat_imu);

	switch (settings.orientation) {
	case ORIENTATION_X_UP:
		// do first-run filter setup
		if (alt_kf.step == 0) {
			rc_vector_zeros(&u, 1);
			rc_vector_zeros(&y, 1);
			alt_kf.x_est.d[0] = bmp_data.alt_m;
			rc_filter_prefill_inputs(&acc_lp, accel_vec[0] - GRAVITY);
			rc_filter_prefill_outputs(&acc_lp, accel_vec[0] - GRAVITY);
		}

		// calculate acceleration and smooth it just a tad
		// put result in u for kalman and flip sign since with altitude, positive
		// is up whereas acceleration in X points up.
		rc_filter_march(&acc_lp, accel_vec[0] - GRAVITY);
		u.d[0] = acc_lp.newest_output;

		// don't bother filtering Barometer, kalman will deal with that
		y.d[0] = bmp_data.alt_m;
		break;
	case (ORIENTATION_Z_DOWN):
		// do first-run filter setup
		if (alt_kf.step == 0) {
			rc_vector_zeros(&u, 1);
			rc_vector_zeros(&y, 1);
			alt_kf.x_est.d[0] = -bmp_data.alt_m;
			rc_filter_prefill_inputs(&acc_lp, accel_vec[2] + GRAVITY);
			rc_filter_prefill_outputs(&acc_lp, accel_vec[2] + GRAVITY);
		}

		// calculate acceleration and smooth it just a tad
		// put result in u for kalman and flip sign since with altitude, positive
		// is up whereas acceleration in Z points down.
		rc_filter_march(&acc_lp, accel_vec[2] + GRAVITY);
		u.d[0] = acc_lp.newest_output;

		// don't bother filtering Barometer, kalman will deal with that
		y.d[0] = -bmp_data.alt_m;
		break;
	default:
		fprintf(stderr, "ERROR: Unknown Body Frame Orientation. Assuming ORIENTATION_X_UP. Please check settings.orientation\n");
		// do first-run filter setup
		if (alt_kf.step == 0) {
			rc_vector_zeros(&u, 1);
			rc_vector_zeros(&y, 1);
			alt_kf.x_est.d[0] = bmp_data.alt_m;
			rc_filter_prefill_inputs(&acc_lp, accel_vec[0] - GRAVITY);
			rc_filter_prefill_outputs(&acc_lp, accel_vec[0] - GRAVITY);
		}

		// calculate acceleration and smooth it just a tad
		// put result in u for kalman and flip sign since with altitude, positive
		// is up whereas acceleration in Z points down.
		rc_filter_march(&acc_lp, accel_vec[0] - GRAVITY);
		u.d[0] = acc_lp.newest_output;

		// don't bother filtering Barometer, kalman will deal with that
		y.d[0] = bmp_data.alt_m;
	}

	rc_kalman_update_lin(&alt_kf, u, y);

	// altitude estimate
	state_estimate.alt_bmp		= alt_kf.x_est.d[0] - events.ground_alt;
	state_estimate.alt_bmp_vel	= alt_kf.x_est.d[1];
	//state_estimate.alt_bmp_accel= alt_kf.x_est.d[2]; //does not work rn (very slow updates)
	state_estimate.alt_bmp_accel = acc_lp.newest_output; //quick, slightly filtered data
	// Estimate apogee altitude:
	__projected_altitude(); //updates state_estimate.proj_ap
	return;
}

static void __feedback_select(void)
{
	state_estimate.roll				= state_estimate.tb_imu[0];
	state_estimate.pitch			= state_estimate.tb_imu[1];
	state_estimate.yaw				= state_estimate.tb_imu[2];
	state_estimate.continuous_yaw	= state_estimate.imu_continuous_yaw;
	state_estimate.continuous_roll	= state_estimate.imu_continuous_roll;

	// If estimating state of the board and using xbee: or other external sources
	if (settings.enable_xbee) {
		state_estimate.X = state_estimate.pos_mocap[0];
		state_estimate.Y = state_estimate.pos_mocap[1];
		state_estimate.Z = state_estimate.pos_mocap[2];
		
		if (settings.use_xbee_roll) {
			state_estimate.roll 	= state_estimate.tb_mocap[0];
		}
		if (settings.use_xbee_pitch) {
			state_estimate.pitch 	= state_estimate.tb_mocap[1];
		}
		if (settings.use_xbee_yaw) {
			state_estimate.yaw 		= state_estimate.tb_mocap[2];
		}
	}
	else if (user_input.use_external_state_estimation) //choose transmitted values, computed externally
	{
		//assume a single source of information for now

		//owerwrite the estimated state on the board with the external data:

		state_estimate.alt_bmp			= fallback.alt; //get the altitude
		state_estimate.alt_bmp_vel		= fallback.alt_vel; //get vertial velocity
		state_estimate.proj_ap			= fallback.proj_ap;
		state_estimate.alt_bmp_accel	= fallback.alt_accel; //get vertical accel
		state_estimate.roll				= fallback.roll;
		state_estimate.pitch			= fallback.pitch;
		state_estimate.yaw				= fallback.yaw;

		if (fallback.flight_state > flight_status)
		{
			flight_status++;
		}
	}
	else {

		switch (settings.orientation)
		{
		case ORIENTATION_X_UP:
			state_estimate.X = state_estimate.alt_bmp;
			state_estimate.Y = state_estimate.pos_mocap[1];
			state_estimate.Z = state_estimate.pos_mocap[2];
			break;
		case ORIENTATION_Z_DOWN:
			state_estimate.X = state_estimate.pos_mocap[0];
			state_estimate.Y = state_estimate.pos_mocap[1];
			state_estimate.Z = -state_estimate.alt_bmp;
			break;
		default:
			fprintf(stderr, "ERROR: Unknown Body Frame Orientation. Assuming ORIENTATION_X_UP. Please check settings.orientation\n");
			state_estimate.X = state_estimate.alt_bmp;
			state_estimate.Y = state_estimate.pos_mocap[1];
			state_estimate.Z = state_estimate.pos_mocap[2];
		}
		
	}

	if (settings.enable_serial)
	{
		if(pick_data_source() != 0) fprintf(stderr,"ERROR: something went wrong in pick_data_source. Failed to overwrite with external input\n");
	}

}

static void __altitude_cleanup(void)
{
	rc_kalman_free(&alt_kf);
	rc_filter_free(&acc_lp);
	return;
}



static void __mocap_check_timeout(void)
{
	if(state_estimate.mocap_running){
		uint64_t current_time = rc_nanos_since_boot();
		// check if mocap data is > 3 steps old
		if((current_time-state_estimate.mocap_timestamp_ns) > (3*1E7)){
			state_estimate.mocap_running = 0;
			if(settings.warnings_en){
				fprintf(stderr,"WARNING, MOCAP LOST VISUAL\n");
			}
		}
	}
	return;
}


int state_estimator_init(void)
{
	__batt_init();
	if(__altitude_init()) return -1;
	state_estimate.initialized = 1;
	return 0;
}

int state_estimator_march(void)
{
	if(state_estimate.initialized==0)
	{
		fprintf(stderr, "ERROR in state_estimator_march, estimator not initialized\n");
		return -1;
	}

	// populate state_estimate struct one setion at a time, top to bottom
	__batt_march();
	__imu_march();
	__mag_march();
	__altitude_march();
	__feedback_select();
	__mocap_check_timeout();

	return 0;
}


int state_estimator_jobs_after_feedback(void)
{
	static int bmp_sample_counter = 0;

	// check if we need to sample BMP this loop
	if(bmp_sample_counter>=BMP_RATE_DIV){
		// perform the i2c reads to the sensor, on bad read just try later
		if(rc_bmp_read(&bmp_data)) return -1;
		bmp_sample_counter=0;
	}
	bmp_sample_counter++;
	return 0;
}


int state_estimator_cleanup(void)
{
	__batt_cleanup();
	__altitude_cleanup();
	return 0;
}
