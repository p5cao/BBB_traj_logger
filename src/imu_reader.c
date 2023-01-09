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
// #define I2C_BUS 2
// #define GPIO_INT_PIN_CHIP 3
// #define GPIO_INT_PIN_PIN  21

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
#include <setpoint_manager.h>
#include <log_manager.h>
#include <settings.h>
#include <mix.h>
#include <thrust_map.h>
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


void __print_usage(void)
{
        printf("\n Options\n");
        printf("-r {rate}       Set sample rate in HZ (default 100)\n");
        printf("                Sample rate must be a divisor of 200\n");
        printf("-m              Enable Magnetometer\n");
        printf("-b              Enable Reading Magnetometer before ISR (default after)\n");
        printf("-c              Show raw compass angle\n");
        printf("-a              Print Accelerometer Data\n");
        printf("-g              Print Gyro Data\n");
        printf("-T              Print Temperature\n");
        printf("-t              Print TaitBryan Angles\n");
        printf("-q              Print Quaternion Vector\n");
        printf("-p {prio}       Set Interrupt Priority and FIFO scheduling policy (requires root)\n");
        printf("-w              Print I2C bus warnings\n");
        printf("-o              Show a menu to select IMU orientation\n");
        printf("-h              Print this help message\n\n");
        return;
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
        printf("%6.1f %6.1f %6.1f |",   data.dmp_TaitBryan[TB_PITCH_X]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG,\
                                                data.dmp_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
        // print accelerometer
        printf(" %5.2f %5.2f %5.2f |",  data.accel[0],\
                                        data.accel[1],\
                                        data.accel[2]);
        //__feedback_isr();
        
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
	
	printf(" %4.2f %4.2f %4.2f |",    fstate.roll, \
                                                fstate.pitch, \
                                                fstate.yaw);
                                        
        //printf(" %4.2f %4.2f %4.2f |",    fstate.roll, \
                                                fstate.pitch, \
                                                fstate.yaw);
        
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
        printf(" Accel XYZ (m/s^2) |");
        printf("\n");
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
        int c, sample_rate, priority;
        int show_something = 0; // set to 1 when any show data option is given.
        // start with default config and modify based on options
        rc_mpu_config_t conf = rc_mpu_default_config();
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
        if(rc_mpu_initialize_dmp(&data, conf)){
                printf("rc_mpu_initialize_failed\n");
                return -1;
        }
        // write labels for what data will be printed and associate the interrupt
        // function to print data immediately after the header.
        __print_header();
        if(!silent_mode) rc_mpu_set_dmp_callback(&__print_data);
        //now just wait, print_data() will be called by the interrupt
        while(running)  rc_usleep(100000);
        // shut things down
        rc_mpu_power_off();
        printf("\n");
        fflush(stdout);
        return 0;
}
