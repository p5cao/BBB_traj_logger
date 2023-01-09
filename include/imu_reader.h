#ifndef IMU_READER_H_INCLUDED
#define IMU_READER_H_INCLUDED
// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
// change these for your platform
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21




#define TWO_PI (M_PI*2.0)

// local functions
rc_mpu_orientation_t __orientation_prompt(void);
rc_mpu_data_t data;
void __print_usage(void);
void __print_data(void);
void __print_header(void);


#endif // IMU_READER_H