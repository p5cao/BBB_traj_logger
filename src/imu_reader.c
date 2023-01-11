/**
 * @file imu_reader.c
 * @example    rc_test_mpu
 *
 * @brief      
 *
 * @author     Pengcheng Cao
 * @date       1/8/2023
 */
 
// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform

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

#include <getopt.h>
#include <signal.h>
#include <stdlib.h> // for atoi() and exit()
#include <rc/mpu.h>
#include <rc/time.h>

#include <imu_reader.h>
#include <feedback.h>

#include <rc_pilot_defs.h>
//#include <setpoint_manager.h>
#include <log_manager.h>
//#include <settings.h>
//#include <mix.h>
//#include <thrust_map.h>
// Global Variables
int running = 0;
int silent_mode = 0;
int show_accel = 1;
int show_gyro  = 1;
int enable_mag = 1;
int show_compass = 0;
int show_temp  = 0;
int show_quat  = 0;
int show_tb = 1;
int orientation_menu = 0;
int accel_transformed = 0;


#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE     200     // hz
#define DT              (1.0/SAMPLE_RATE)
#define ACCEL_LP_TC     20*DT   // fast LP filter for accel
#define PRINT_HZ        10
#define BMP_RATE_DIV    10      // optionally sample bmp less frequently than mpu
#define OVERSAMPLE  BMP_OVERSAMPLE_16
#define INTERNAL_FILTER BMP_FILTER_OFF



//static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
// static rc_kalman_t kf_x = RC_KALMAN_INITIALIZER;
// static rc_kalman_t kf_y = RC_KALMAN_INITIALIZER;
static rc_kalman_t kf = RC_KALMAN_INITIALIZER;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
// static rc_filter_t acc_lp_x = RC_FILTER_INITIALIZER;
// static rc_filter_t acc_lp_y = RC_FILTER_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
static rc_filter_t x_acc_lp = RC_FILTER_INITIALIZER;
static rc_filter_t y_acc_lp = RC_FILTER_INITIALIZER;


// static int est_kf;
// // altitude kalman filer elements
// //static rc_filter_t D_roll, D_pitch, D_yaw, D_batt, D_altitude
//static rc_filter_t D_batt, altitude_lp;
// static rc_matrix_t F, G, H, Q, R, Pi;
// static rc_kalman_t kf;
// static rc_vector_t u,y;
// static rc_filter_t acc_lp;
static rc_filter_t altitude_lp = RC_FILTER_INITIALIZER;

//static int logging_enabled = 1;


FILE *flight_log;

log_entry_t new_log;

// static int __estimate_altitude()
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
// 	//fstate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
// 	fstate.altitude_kf = kf.x_est.d[0];
// 	fstate.alt_kf_vel = kf.x_est.d[1];
// 	fstate.alt_kf_accel = kf.x_est.d[2];

// 	return 0;
// }

static void __transform(void)
{
	int i, j;
	double accel[3];
	double quat[4];
	for(i=0;i<3;i++) accel[i]= data.accel[i];
	//for(j=0;j<4;i++) quat[i]= data.dmp_quat[i];
	
	rc_quaternion_rotate_vector_array(accel, data.dmp_quat);
	// fstate.x_accel= accel[0];
	// fstate.y_accel= accel[1];
	// fstate.z_accel= accel[2];
	
	//dt = 1.0/settings.feedback_hz;
	
	//rc_mpu_config_t mpu_conf;
        rc_matrix_t F = RC_MATRIX_INITIALIZER;
        rc_matrix_t G = RC_MATRIX_INITIALIZER;
        rc_matrix_t H = RC_MATRIX_INITIALIZER;
        rc_matrix_t Q = RC_MATRIX_INITIALIZER;
        rc_matrix_t R = RC_MATRIX_INITIALIZER;
        rc_matrix_t Pi = RC_MATRIX_INITIALIZER;
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
        if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return ;
        // initialize the little LP filter to take out accel noise
        if(rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return;

         // init barometer and read in first data
        //printf("initializing barometer\n");
        // if(rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) return;
        // if(rc_bmp_read(&bmp_data)) return;

        
        // // filtering x coordinates
        // if(kf_x.step==0){
        //         kf_x.x_est.d[0] = 0.0;
        //         rc_filter_prefill_inputs(&acc_lp_x, accel[0]);
        //         rc_filter_prefill_outputs(&acc_lp_x, accel[0]);
        // }
        // // calculate acceleration and smooth it just a tad
        // rc_filter_march(&acc_lp_x, accel[0]);
        // u.d[2] = acc_lp_x.newest_output;
        // // // // don't bother filtering Barometer, kalman will deal with that
        // y.d[0] = 0.0;
        // if(rc_kalman_update_lin(&kf_x, u, y)) running=0;
        
        // fstate.x = kf_x.x_est.d[0];
        // fstate.x_vel = kf_x.x_est.d[1];
        fstate.x_accel = accel[0];
        fstate.y_accel = accel[1];
        fstate.z_accel = accel[2];

        //  // filtering y coordinates
        // if(kf_y.step==0){
        //         kf_y.x_est.d[0] = 0.0;
        //         rc_filter_prefill_inputs(&acc_lp_y, accel[1]);
        //         rc_filter_prefill_outputs(&acc_lp_y, accel[1]);
        // }
        // // calculate acceleration and smooth it just a tad
        // rc_filter_march(&acc_lp_y, accel[1]);
        // // u.d[0] = acc_lp.newest_output;
        // // // don't bother filtering Barometer, kalman will deal with that
        // // y.d[0] = bmp_data.alt_m;
        
        // fstate.y = kf_y.x_est.d[0];
        // fstate.y_vel = kf_y.x_est.d[1];
        // fstate.y_accel = kf_y.x_est.d[2];
        
        static int bmp_sample_counter = 0;
        // filtering z coordinates
        // if(kf.step==0){
        //         kf.x_est.d[0] = bmp_data.alt_m;
        //         rc_filter_prefill_inputs(&acc_lp, accel[2]-9.80665);
        //         rc_filter_prefill_outputs(&acc_lp, accel[2]-9.80665);
        // }
        // // calculate acceleration and smooth it just a tad
        // rc_filter_march(&acc_lp, accel[2]-9.80665);
        // u.d[0] = acc_lp.newest_output;
        // // don't bother filtering Barometer, kalman will deal with that
        // y.d[0] = bmp_data.alt_m;
        // if(rc_kalman_update_lin(&kf, u, y)) running=0;

        // fstate.z_accel = kf.x_est.d[2]+9.80665;
        // fstate.z_vel = kf.x_est.d[1];
        // fstate.z = kf.x_est.d[0];
        
	//fflush(stdout);
	// do first-run filter setup
// 	if(kf.step==0){
// 		kf.x_est.d[0] = bmp_data.alt_m;
// 		rc_filter_prefill_inputs(&acc_lp, accel[2]-9.80665);
// 		rc_filter_prefill_outputs(&acc_lp, accel[2]-9.80665);
// 	}

// 	// calculate acceleration and smooth it just a tad
// 	rc_filter_march(&acc_lp, accel[2]-9.80665);
// 	u.d[0] = acc_lp.newest_output;

// 	// don't bother filtering Barometer, kalman will deal with that
// 	y.d[0] = bmp_data.alt_m;
// 	if (rc_kalman_update_lin(&kf, u, y)) running = 0;

// 	// altitude estimate
// 	//fstate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
	
//         //now check if we need to sample BMP this loop
//         bmp_sample_counter++;
//         if(bmp_sample_counter>=BMP_RATE_DIV){
//                 // perform the i2c reads to the sensor, on bad read just try later
//                 if(rc_bmp_read(&bmp_data)) return;
//                 bmp_sample_counter=0;
//         }
// 	//fstate.altitude_kf = kf.x_est.d[0];
//         fstate.altitude_kf = kf.x_est.d[0];
// 	fstate.alt_kf_vel = kf.x_est.d[1];
// 	fstate.alt_kf_accel = kf.x_est.d[2];
	
        
	return;
}


static int __Kalman_filtering(){
        int i;
	double accel_vec[3];
	double x_accel_filtered;
	double y_accel_filtered;
	//double z_accel_filtered;
	accel_vec[0] = fstate.x_accel;
	accel_vec[1] = fstate.y_accel;
	accel_vec[2] = fstate.z_accel;
        // do first-run filter setup
        if(rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return 0;
        if(rc_filter_first_order_lowpass(&x_acc_lp, DT, ACCEL_LP_TC)) return 0;
        if(rc_filter_first_order_lowpass(&y_acc_lp, DT, ACCEL_LP_TC)) return 0;
	if(kf.step==0){
		kf.x_est.d[0] = bmp_data.alt_m;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
		rc_filter_prefill_inputs(&x_acc_lp, accel_vec[0]);
		rc_filter_prefill_outputs(&x_acc_lp, accel_vec[0]);
		rc_filter_prefill_inputs(&y_acc_lp, accel_vec[1]);
		rc_filter_prefill_outputs(&y_acc_lp, accel_vec[1]);
	}
	
	// calculate acceleration and smooth it just a tad
	rc_filter_march(&x_acc_lp, accel_vec[0]);
	rc_filter_march(&y_acc_lp, accel_vec[1]);
	rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
	u.d[0] = acc_lp.newest_output;
	x_accel_filtered = x_acc_lp.newest_output;
	y_accel_filtered = y_acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
//y.d[0] = bmp_data.alt_m;
        y.d[0] = 0.0;
	rc_kalman_update_lin(&kf, u, y);

// 	// altitude estimate
// 	fstate.altitude_bmp = rc_filter_march(&altitude_lp,bmp_data.alt_m);
// 	fstate.altitude_kf = kf.x_est.d[0];
// 	fstate.alt_kf_vel = kf.x_est.d[1];
// 	fstate.alt_kf_accel = kf.x_est.d[2];
	// x, y accel estimate
	fstate.x_accel = x_accel_filtered;
	fstate.y_accel = y_accel_filtered;
	return 0;
}
/**
 * This is the IMU interrupt function.
 */
void __print_data(void)
{

        printf("\r");
        printf(" ");
        printf(" %4.2f %4.2f %4.2f %4.2f |",    data.dmp_quat[QUAT_W], \
                                                data.dmp_quat[QUAT_X], \
                                                data.dmp_quat[QUAT_Y], \
                                                data.dmp_quat[QUAT_Z]);
        // print TaitBryan angles
        printf("%6.1f %6.1f %6.1f |",   data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                        data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                        data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
        // print accelerometer
        printf(" %5.2f %5.2f %5.2f |",  fstate.x_accel,\
                                        fstate.x_vel,\
                                        fstate.x);
        printf(" %5.2f %5.2f %5.2f |",  fstate.y_accel,\
                                        fstate.y_vel,\
                                        fstate.y);                              
        printf(" %5.2f %5.2f %5.2f |",  fstate.z_accel,\
                                        fstate.z_vel,\
                                        fstate.z);

        double tmp, yaw_reading;

	if(fstate.initialized==0){
		fprintf(stderr, "ERROR in feedback_state_estimate, feedback controller not initialized\n");
		return;
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
	
	tmp = -yaw_reading + (num_yaw_spins * TWO_PI);
	//detect the crossover point at +-PI and write new value to core state
	if(tmp-last_yaw < -M_PI) num_yaw_spins++;
	else if (tmp-last_yaw > M_PI) num_yaw_spins--;
	// finally num_yaw_spins is updated and the new value can be written
	fstate.yaw = -yaw_reading + (num_yaw_spins * TWO_PI);
	last_yaw = fstate.yaw;

	fprintf(flight_log,"\n");
	
        fprintf(flight_log,  "%+5.2f |%+5.2f |%+5.2f |",\
			        fstate.x_accel,\
				fstate.y_accel,\
				fstate.z_accel);
				//fstate.alt_kf_accel);
	fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|",\
				fstate.roll,\
				fstate.pitch,\
				fstate.yaw);
				
	fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|",\
			fstate.x_vel,\
			fstate.y_vel,\
			fstate.z_vel);
			//fstate.alt_kf_vel);
			
	fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|",\
		fstate.x,\
		fstate.y,\
		fstate.z);
		//fstate.altitude_kf);
	fprintf(flight_log,"\n");
        
        fflush(stdout);
        return;
}
/**
 * Based on which data is marked to be printed, print the correct labels. this
 * is printed only once and the actual data is updated on the next line.
 */
void __print_header(void)
{
        printf(" ");
        printf("    DMP Quaternion   |");
        printf(" DMP TaitBryan (deg) |");
        // printf(" Accel XYZ (m/s^2) |");
        // printf(" UAV rpy states (rad) |");
        printf(" acc_x | vel_x | X |");
        printf(" acc_y | vel_y | Y |");
        printf(" acc_Z | vel_z | Z |");
        printf("\n");
        //int logged = log_manager_init();
        
        // turn off linewrap to avoid runaway prints
	//printf(WRAP_DISABLE);
	flight_log = fopen("flight_log.txt","w");
	// log the date
	time_t now = time(NULL);
        time (&now);
	fprintf(flight_log, "This flight happened on: %s\n",ctime(&now));
	// print the header
	//__reset_colour();
	// if(logging_enabled){
	fprintf(flight_log, "acc_x |acc_y|acc_z|");
	fprintf(flight_log, " roll |pitch| yaw |");
	fprintf(flight_log, " x_vel|y_vel|z_vel|");
	fprintf(flight_log, "  X   | Y   |  Z  |");
	// }
}
/**
 * @brief      interrupt handler to catch ctrl-c
 */
void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}
/**
 * If the user selects the -o option for orientation selection, this menu will
 * displayed to prompt the user for which orientation to use. It will return a
 * valid rc_mpu_orientation_t when a number 1-6 is given or quit when 'q' is
 * pressed. On other inputs the user will be allowed to enter again.
 *
 * @return     the orientation enum chosen by user
 */
/**
 * main() serves to parse user options, initialize the imu and interrupt
 * handler, and wait for the rc_get_state()==EXITING condition before exiting
 * cleanly. The imu_interrupt function print_data() is what actually prints new
 * imu data to the screen after being set with rc_mpu_set_dmp_callback().
 *
 * @param[in]  argc  The argc
 * @param      argv  The argv
 *
 * @return     0 on success -1 on failure
 */


int main(int argc, char *argv[])
{
        rc_bmp_data_t bmp_data;
        int c, sample_rate, priority;
        int show_something = 0; // set to 1 when any show data option is given.
        // start with default config and modify based on options
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.dmp_sample_rate = SAMPLE_RATE;
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
        conf.dmp_fetch_accel_gyro=1;
        fstate.initialized = 1;
        // parse arguments
        opterr = 0;
        // set signal handler so the loop can exit cleanly
        signal(SIGINT, __signal_handler);
        running = 1;
        // now set up the imu for dmp interrupt operation

        // init barometer and read in first data
        printf("initializing barometer\n");
        if(rc_bmp_init(OVERSAMPLE, INTERNAL_FILTER)) return -1;
        if(rc_bmp_read(&bmp_data)) return -1;
        
        if(rc_mpu_initialize_dmp(&data, conf)){
        printf("rc_mpu_initialize_failed\n");
        return -1;
        }
        // write labels for what data will be printed and associate the interrupt
        // function to print data immediately after the header.
        __print_header();
        //if(!silent_mode) rc_mpu_set_dmp_callback(&__transform);
        //now just wait, print_data() will be called by the interrupt
        while(running) {
                //__estimate_altitude();
        	__transform();
        	__Kalman_filtering();
        	__print_data();
        	fflush(stdout);
        	rc_usleep(100000);
        }
        	
        // shut things down
        rc_mpu_power_off();
        printf("\n");
        fflush(stdout);
        return 0;
}
