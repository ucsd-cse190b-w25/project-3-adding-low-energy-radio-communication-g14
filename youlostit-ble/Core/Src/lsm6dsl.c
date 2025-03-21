/*
 * lsm6dsl.c
 *
 *  Created on: Feb 4, 2025
 *      Author: ibrah
 */

#include "lsm6dsl.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

// Initialize the LSM6DSL accelerometer and gyroscope

void lsm6dsl_init() {
    uint8_t who_am_i = 0;
    uint8_t reg = LSM6DSL_WHO_AM_I;

    // Add a small delay before starting
//    for (volatile int i = 0; i < 100000; i++);

    // Verify the device by reading WHO_AM_I
    if (i2c_transaction(LSM6DSL_ADDRESS, 0, &reg, 1) != 0) {
        //printf("Error: Failed to send WHO_AM_I register address!\n");
        return;
    }

    if (i2c_transaction(LSM6DSL_ADDRESS, 1, &who_am_i, 1) != 0) {
        //printf("Error: Failed to read WHO_AM_I register!\n");
        return;
    }

    //printf("LSM6DSL WHO_AM_I: 0x%02X\n", who_am_i);
    if (who_am_i != 0x6A) {
        //printf("Error: LSM6DSL not detected!\n");
        return;
    }

    // Add a small delay
    for (volatile int i = 0; i < 100000; i++);

    // Configure accelerometer first - ODR = 104 Hz (0x40), ±2g (0x00)
    uint8_t accel_config[2] = {LSM6DSL_CTRL1_XL, 0x40};
    if (i2c_transaction(LSM6DSL_ADDRESS, 0, accel_config, 2) != 0) {
        //printf("Error: Failed to configure accelerometer\n");
        return;
    }

    // Add a small delay
    for (volatile int i = 0; i < 100000; i++);

    // Enable BDU and IF_INC
    uint8_t ctrl3_config[2] = {LSM6DSL_CTRL3_C, 0x44};
    if (i2c_transaction(LSM6DSL_ADDRESS, 0, ctrl3_config, 2) != 0) {
        //printf("Error: Failed to configure CTRL3_C\n");
        return;
    }

    //printf("LSM6DSL Initialized Successfully!\n");
}


// Read acceleration data from X, Y, Z axes
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t reg = LSM6DSL_OUTX_L_XL;
    uint8_t raw_data[6];

    // Set register address
    if (i2c_transaction(LSM6DSL_ADDRESS, 0, &reg, 1) != 0) {
        //printf("Error: Failed to set register address for reading!\n");
        return;
    }

    // Read all 6 bytes in a single transaction
    if (i2c_transaction(LSM6DSL_ADDRESS, 1, raw_data, 6) != 0) {
        //printf("Error: Failed to read acceleration data!\n");
        return;
    }

    // Combine bytes and convert to signed 16-bit integers
    *x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    *y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    *z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Convert to mg (milli-g) values
    // With ±2g full scale, 1 mg = 0.061 mg/LSB
    *x = (*x * 61) / 1000;  // Convert to mg
    *y = (*y * 61) / 1000;  // Convert to mg
    *z = (*z * 61) / 1000;  // Convert to mg
}

