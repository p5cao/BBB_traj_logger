#ifndef IMU_READER_H_INCLUDED
#define IMU_READER_H_INCLUDED
// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21

// #include <inttypes.h>

// #define LOG_TABLE \
// 	X(uint64_t,	"%" PRIu64,	loop_index	) \
// 	X(uint64_t,	"%" PRIu64,	last_step_ns	) \
// 							  \
// 	X(double,	"%f",		altitude_kf	) \
// 	X(double,	"%f",		altitude_bmp)	  \
// 	X(double,	"%f",		roll		) \
// 	X(double,	"%f",		pitch		) \
// 	X(double,	"%f",		yaw		) \
// 							  \
// 	X(double,	"%f",		Z_throttle_sp	) \
// 	X(double,	"%f",		altitude_sp	) \
// 	X(double,	"%f",		roll_sp		) \
// 	X(double,	"%f",		pitch_sp	) \
// 	X(double,	"%f",		yaw_sp		) \
// 							\
// 	X(double,	"%f",		u_X		) \
// 	X(double,	"%f",		u_Y		) \
// 	X(double,	"%f",		u_Z		) \
// 	X(double,	"%f",		u_roll		) \
// 	X(double,	"%f",		u_pitch		) \
// 	X(double,	"%f",		u_yaw		) \
// 							  \
// 	X(double,	"%f",		mot_1		) \
// 	X(double,	"%f",		mot_2		) \
// 	X(double,	"%f",		mot_3		) \
// 	X(double,	"%f",		mot_4		) \
// 	X(double,	"%f",		mot_5		) \
// 	X(double,	"%f",		mot_6		) \
// 	X(double,	"%f",		v_batt		)

// typedef struct log_entry_t { LOG_TABLE } log_entry_t;
// #undef X

#define TWO_PI (M_PI*2.0)

// local functions
rc_mpu_orientation_t __orientation_prompt(void);
rc_mpu_data_t data;

static void __transform(void);
static int __estimate_altitude();
static int __complement_filtering();
static int __integrate();

void __print_usage(void);
void __print_data(void);
void __print_header(void);


#endif // IMU_READER_H