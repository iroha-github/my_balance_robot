#ifndef _MPU6050_I2C_H_
#define _MPU6050_I2C_H_

#include <stdint.h>
#include "madgwick_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

// MPU6050の初期化
void mpu6050_reset(void);

// MPU6050から加速度, ジャイロ, 温度の生データ(16bit)を取得
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

// MPU6050の加速度/ジャイロのオフセットを計測(静止・水平な状態で使用)
void mpu6050_calibrate(float *accel_offset, float *gyro_offset, int num_samples);

// オフセット補正後の値を取得
void mpu6050_adjusted_values(float adj_accel[3], float adj_gyro[3], float accel_offset[3], float gyro_offset[3]);
#ifdef __cplusplus
}
#endif

#endif // _MPU6050_I2C_H_
