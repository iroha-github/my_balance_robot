#ifndef _MPU6050_I2C_H_
#define _MPU6050_I2C_H_

#include <stdint.h>
#include "madgwick_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_rad;
    float gy_rad;
    float gz_rad;
} SensorData_t;

void mpu6050_reset(void);
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
void mpu6050_calibrate(float *accel_offset, float *gyro_offset, int num_samples);
void mpu6050_adjusted_values(SensorData_t* data, float accel_offset[3], float gyro_offset[3]);

#ifdef __cplusplus
}
#endif

#endif // _MPU6050_I2C_H_
