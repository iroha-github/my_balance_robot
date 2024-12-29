#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "mpu6050_i2c.h"      // ヘッダ
// Madgwickフィルタヘッダは「補正演算のために」読み込んでもOKだが、
// ここではオフセット取得だけなので必須でなければなくてもよい。

// MPU6050のI2Cアドレス (0x68)
static const int MPU6050_ADDR = 0x68;

// ±2g, ±250deg/s 設定の場合のLSB
#define ACCEL_LSB_2G 16384.0f
#define GYRO_LSB_250 131.0f

#define I2C_PORT i2c1

//---------------------------------------------------------------------------------
// MPU6050 初期化/リセット
//---------------------------------------------------------------------------------
void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x80};  // PWR_MGMT_1: デバイスリセット
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(150);

    // スリープ解除
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false); 
    sleep_ms(10);

    // ジャイロレンジ ±250 deg/s
    uint8_t gyro_config[2] = {0x1B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    // 加速度レンジ ±2g
    uint8_t accel_config[2] = {0x1C, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);
}

//---------------------------------------------------------------------------------
// 生データ読み取り
//---------------------------------------------------------------------------------
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t* temp) {
    uint8_t reg = 0x3B;  // ACCEL_XOUT_H
    uint8_t buffer[6];

    // 加速度 (0x3B~0x40)
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // ジャイロ (0x43~0x48)
    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // 温度(0x41~0x42)
    reg = 0x41;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

//---------------------------------------------------------------------------------
// キャリブレーション (静止・水平)
//---------------------------------------------------------------------------------
void mpu6050_calibrate(float* accel_offset, float* gyro_offset, int num_samples) {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    // 安定待ち
    sleep_ms(200);

    for (int i = 0; i < num_samples; i++) {
        int16_t accel_raw[3], gyro_raw[3], temp;
        mpu6050_read_raw(accel_raw, gyro_raw, &temp);

        ax_sum += accel_raw[0];
        ay_sum += accel_raw[1];
        az_sum += accel_raw[2];
        gx_sum += gyro_raw[0];
        gy_sum += gyro_raw[1];
        gz_sum += gyro_raw[2];

        sleep_ms(5);
    }

    // 平均
    float ax_avg = (float)ax_sum / num_samples;
    float ay_avg = (float)ay_sum / num_samples;
    float az_avg = (float)az_sum / num_samples;
    float gx_avg = (float)gx_sum / num_samples;
    float gy_avg = (float)gy_sum / num_samples;
    float gz_avg = (float)gz_sum / num_samples;

    // 加速度オフセット計算(±2g設定時)
    accel_offset[0] = ax_avg;                // x => 0gが理想
    accel_offset[1] = ay_avg;                // y => 0gが理想
    accel_offset[2] = az_avg - 16384.0f;     // z => +1g(=16384LSB)が理想

    // ジャイロオフセット(±250deg/s設定時)
    // ジャイロraw [LSB], 0が理想
    gyro_offset[0] = gx_avg;
    gyro_offset[1] = gy_avg;
    gyro_offset[2] = gz_avg;
}
