/**
 * @file logger.c
 *
 * Author: Pengcheng Cao
 */
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <string.h>

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
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/pthread.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()

#include <feedback.h>
#include <rc_pilot_defs.h>
#include <setpoint_manager.h>
#include <log_manager.h>
#include <settings.h>
#include <mix.h>
#include <thrust_map.h>
#include <imu_reader.h>

uav_state_t fstate; // extern variable in feedback.h




// altitude kalman filer elements
//static rc_filter_t D_roll, D_pitch, D_yaw, D_batt, D_altitude
static rc_filter_t D_batt, altitude_lp;
static rc_matrix_t F, G, H, Q, R, Pi;
static rc_kalman_t kf;
static rc_vector_t u,y;
static rc_filter_t acc_lp;

// local functions
//static void __feedback_isr(void);
static double __batt_voltage();
//static int __estimate_altitude();
static int __feedback_state_estimate();

// array of two buffers so one can fill while writing the other to file
log_entry_t buffer[2][BUF_LEN];

int logging_enabled = 1;

static rc_bmp_data_t bmp_data;


static double __batt_voltage()
{
	float tmp;

	//tmp = rc_adc_dc_jack();
	//if(tmp<3.0f) tmp = settings.v_nominal;
	return tmp;
}

static int __init_altitude_kf()
{
//initialize altitude kalman filter and bmp sensor
	F = rc_matrix_empty();
	G = rc_matrix_empty();
	H = rc_matrix_empty();
	Q = rc_matrix_empty();
	R = rc_matrix_empty();
	Pi = rc_matrix_empty();
	u = rc_vector_empty();
	y = rc_vector_empty();
	kf = rc_kalman_empty();
	acc_lp = rc_filter_empty();
	altitude_lp = rc_filter_empty();

	int Nx = 3;
	int Ny = 1;
	int Nu = 1;
	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = dt;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -dt; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5*dt*dt;
	G.d[0][1] = dt;
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
	if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return -1;
	// initialize the little LP filter to take out accel noise
	if(rc_filter_first_order_lowpass(&acc_lp, dt, 20*dt)) return -1;

	// initialize a LP on baromter for comparison to KF
	if(rc_filter_butterworth_lowpass(&altitude_lp, 2, dt, ALT_CUTOFF_FREQ)) return -1;

	// init barometer and read in first data
	if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))	return -1;
	if(rc_bmp_read(&bmp_data)) return -1;
	rc_filter_prefill_inputs(&altitude_lp, bmp_data.alt_m);
	rc_filter_prefill_outputs(&altitude_lp, bmp_data.alt_m);

	return 0;
}

static int __feedback_state_estimate()
{
	double tmp, yaw_reading;

	if(fstate.initialized==0){
		fprintf(stderr, "ERROR in feedback_state_estimate, feedback controller not initialized\n");
		return -1;
	}

	// collect new IMU roll/pitch data
	// if(settings.enable_magnetometer){
	// 	fstate.roll  = data.fused_TaitBryan[TB_ROLL_Y];
	// 	fstate.pitch = data.fused_TaitBryan[TB_PITCH_X];
	// 	yaw_reading  = data.fused_TaitBryan[TB_YAW_Z];

	// }
	// else{
	fstate.roll  = data.dmp_TaitBryan[TB_ROLL_Y];
	fstate.pitch = data.dmp_TaitBryan[TB_PITCH_X];
	yaw_reading  = data.dmp_TaitBryan[TB_YAW_Z];
	//}

	fstate.roll_rate = data.gyro[0];
	fstate.pitch_rate = data.gyro[1];
	fstate.yaw_rate = -data.gyro[2];
    //dt = 1.0/settings.feedback_hz;
	// yaw is more annoying since we have to detect spins
	// also make sign negative since NED coordinates has Z point down
	tmp = -yaw_reading + (num_yaw_spins * TWO_PI);
	//detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	fstate.yaw = -yaw_reading + (num_yaw_spins * TWO_PI);
	last_yaw = fstate.yaw;

	// filter battery voltage.
	//fstate.v_batt = rc_filter_march(&D_batt,__batt_voltage());

	return 0;
}


// int __estimate_altitude()
// {
// 	int i;
// 	double accel_vec[3];
// 	static int bmp_sample_counter = 0;

// 	// check if we need to sample BMP this loop
// 	if(bmp_sample_counter>=BMP_RATE_DIV){
// 		// perform the i2c reads to the sensor, on bad read just try later
// 		if(rc_bmp_read(&bmp_data)) return -1;
// 		bmp_sample_counter=0;
// 	}
// 	bmp_sample_counter++;

// 	// make copy of acceleration reading before rotating
// 	for(i=0;i<3;i++) accel_vec[i]= data.accel[i];
// 	// rotate accel vector
// 	rc_quaternion_rotate_vector_array(accel_vec,data.dmp_quat);

// 	// do first-run filter setup
// 	if(kf.step==0){
// 		kf.x_est.d[0] = bmp_data.alt_m;
// 		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
// 		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
// 	}

// 	// calculate acceleration and smooth it just a tad
// 	rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
// 	u.d[0] = acc_lp.newest_output;

// 	// don't bother filtering Barometer, kalman will deal with that
// 	y.d[0] = bmp_data.alt_m;
// 	rc_kalman_update_lin(&kf, u, y);

// 	// altitude estimate
// 	fstate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
// 	fstate.altitude_kf = kf.x_est.d[0];
// 	fstate.alt_kf_vel = kf.x_est.d[1];
// 	fstate.alt_kf_accel = kf.x_est.d[2];

// 	return 0;
// }

// void __feedback_isr(void)
// {
// 	//setpoint_manager_update();
// 	__feedback_state_estimate();
// 	__estimate_altitude();
// 	//__feedback_control();
// }

int log_manager_init()
{
	int i;
	char path[100];
	struct stat st = {0};

	// if the thread if running, stop before starting a new log file
	if(logging_enabled){
		//fprintf(stderr,"ERROR: in start_log_manager, log manager already running.\n");
		//return -1;
		log_manager_cleanup();
	}

	// first make sure the directory exists, make it if not
	if (stat(LOG_DIR, &st) == -1) {
		mkdir(LOG_DIR, 0755);
	}

	// search for existing log files to determine the next number in the series
	for(i=1;i<=MAX_LOG_FILES+1;i++){
		memset(&path, 0, sizeof(path));
		sprintf(path, LOG_DIR "%d.csv", i);
		// if file exists, move onto the next index
		if(stat(path, &st)==0) continue;
		else break;
	}
	// limit number of log files
	if(i==MAX_LOG_FILES+1){
		printf("ERROR: log file limit exceeded\n");
		printf("delete old log files before continuing\n");
		return -1;
	}
	// create and open new file for writing
	fd = fopen(path, "w+");
	if(fd == 0) {
		printf("ERROR: can't open log file for writing\n");
		return -1;
	}
	// write header
	#define X(type, fmt, name) fprintf(fd, "%s," , #name);
	LOG_TABLE
	#undef X
	fprintf(fd, "\n");
	fflush(fd);

	// start thread
	logging_enabled = 1;
	num_entries = 0;
	buffer_pos = 0;
	current_buf = 0;
	needs_writing = 0;

	// start logging thread
	if(rc_pthread_create(&pthread, __log_manager_func, NULL, SCHED_FIFO, LOG_MANAGER_PRI)<0){
		fprintf(stderr,"ERROR in start_log_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(1000);
	return 0;
}

int log_manager_cleanup()
{
	// just return if not logging
	if(logging_enabled==0) return 0;

	// disable logging so the thread can stop and start multiple times
	// thread also exits on rc_get_state()==EXITING
	logging_enabled=0;
	int ret = rc_pthread_timed_join(pthread,NULL,LOG_MANAGER_TOUT);
	if(ret==1) fprintf(stderr,"WARNING: log_manager_thread exit timeout\n");
	else if(ret==-1) fprintf(stderr,"ERROR: failed to join log_manager thread\n");
	return ret;
}

void* __log_manager_func(__attribute__ ((unused)) void* ptr)
{
	int i, buf_to_write;
	// while logging enabled and not exiting, write full buffers to disk
	while(rc_get_state()!=EXITING && logging_enabled){
		if(needs_writing){
			// buffer to be written is opposite of one currently being filled
			if(current_buf==0) buf_to_write=1;
			else buf_to_write=0;
			// write the full buffer to disk;
			for(i=0;i<BUF_LEN;i++){
				__write_log_entry(buffer[buf_to_write][i]);
			}
			fflush(fd);
			needs_writing = 0;
		}
		rc_usleep(1000000/LOG_MANAGER_HZ);
	}

	// if program is exiting or logging got disabled, write out the rest of
	// the logs that are in the buffer current being filled
	//printf("writing out remaining log file\n");
	for(i=0;i<buffer_pos;i++){
		__write_log_entry(buffer[current_buf][i]);
	}
	fflush(fd);
	fclose(fd);
	//printf("log file closed\n");
	// zero out state
	logging_enabled = 0;
	num_entries = 0;
	buffer_pos = 0;
	current_buf = 0;
	needs_writing = 0;
	return NULL;
}

int __write_log_entry(log_entry_t entry)
{
	#define X(type, fmt, name) fprintf(fd, fmt "," , entry.name);
	LOG_TABLE
	#undef X
	fprintf(fd, "\n");
	return 0;
}

int add_log_entry(log_entry_t new)
{
	if(!logging_enabled){
		fprintf(stderr,"ERROR: trying to log entry while logger isn't running\n");
		return -1;
	}
	if(needs_writing && buffer_pos >= BUF_LEN){
		fprintf(stderr,"WARNING: logging buffer full, skipping log entry\n");
		return -1;
	}
	// add to buffer and increment counters
	buffer[current_buf][buffer_pos] = new;
	buffer_pos++;
	num_entries++;
	// check if we've filled a buffer
	if(buffer_pos >= BUF_LEN){
		buffer_pos = 0;		// reset buffer position to 0
		needs_writing = 1;	// flag the writer to dump to disk
		// swap buffers
		if(current_buf==0) current_buf=1;
		else current_buf=0;
	}
	return 0;
}