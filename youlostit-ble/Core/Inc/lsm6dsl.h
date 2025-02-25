
#ifndef LSM_H_
#define LSM_H_
#include <stdint.h>
// Register addresses
#define LSM6DSL_WHO_AM_I    0x0F  // Device ID register (should return 0x6A)
#define LSM6DSL_CTRL1_XL    0x10  // Accelerometer control
#define LSM6DSL_CTRL2_G     0x11  // Gyroscope control
#define LSM6DSL_CTRL3_C     0x12  // Control register for general settings (BDU, IF_INC)
#define LSM6DSL_OUTX_L_XL   0x28  // X-axis acceleration data low byte
#define LSM6DSL_OUTX_H_XL   0x29  // X-axis acceleration data high byte
#define LSM6DSL_OUTY_L_XL   0x2A  // Y-axis acceleration data low byte
#define LSM6DSL_OUTY_H_XL   0x2B  // Y-axis acceleration data high byte
#define LSM6DSL_OUTZ_L_XL   0x2C  // Z-axis acceleration data low byte
#define LSM6DSL_OUTZ_H_XL   0x2D  // Z-axis acceleration data high byte
#define LSM6DSL_ADDRESS		0X6A  // accelerometer address

void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);


#endif /* LSM_H_ */
