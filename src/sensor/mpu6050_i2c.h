#ifndef _MPU6050_I2C_H_
#define _MPU6050_I2C_H_

#include <stdint.h>
#include "madgwick_filter.h"  // (for additional definitions if needed)

#ifdef __cplusplus
extern "C" {
#endif

// Structure to hold processed sensor data
typedef struct {
    float ax_g;    // Accelerometer X-axis in g
    float ay_g;    // Accelerometer Y-axis in g
    float az_g;    // Accelerometer Z-axis in g
    float gx_rad;  // Gyroscope X-axis in rad/s
    float gy_rad;  // Gyroscope Y-axis in rad/s
    float gz_rad;  // Gyroscope Z-axis in rad/s
} SensorData_t;

/**
 * @brief Reset and initialize the MPU6050.
 */
void mpu6050_reset(void);

/**
 * @brief Read raw 16-bit sensor values (accelerometer, gyroscope, and temperature).
 */
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

/**
 * @brief Calibrate the MPU6050 sensor offsets.
 * @param accel_offset Array to store accelerometer offsets (3 values)
 * @param gyro_offset  Array to store gyroscope offsets (3 values)
 * @param num_samples  Number of samples to average during calibration
 */
void mpu6050_calibrate(float *accel_offset, float *gyro_offset, int num_samples);

/**
 * @brief Obtain sensor values with offsets removed and additional filtering applied.
 * @param data         Pointer to a SensorData_t structure where results are stored.
 * @param accel_offset Previously computed accelerometer offsets.
 * @param gyro_offset  Previously computed gyroscope offsets.
 */
void mpu6050_adjusted_values(SensorData_t *data, float accel_offset[3], float gyro_offset[3]);

#ifdef __cplusplus
}
#endif

#endif // _MPU6050_I2C_H_
