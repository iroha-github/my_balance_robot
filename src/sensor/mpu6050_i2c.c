#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "mpu6050_i2c.h"
#include "config.h"

// MPU6050 I2C address (0x68)
static const int MPU6050_ADDR = 0x68;

// Accelerometer and gyro sensitivity (for ±2g and ±250deg/s)
#define ACCEL_LSB_2G 16384.0f
#define GYRO_LSB_250 131.0f

// Uncomment the following line to enable debug output for calibration/testing
// #define DEBUG_mpu

//------------------------------------------------------------------------------
// MPU6050 Reset and Initialization with DLPF configuration
//------------------------------------------------------------------------------
void mpu6050_reset() {
    // Reset the MPU6050
    uint8_t buf[] = {0x6B, 0x80};  // PWR_MGMT_1: set reset bit
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(150);

    // Wake up the device (clear sleep bit)
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(10);

    // --- New Step: Configure the Digital Low Pass Filter (DLPF) ---
    // Write to the CONFIG register (0x1A). The value 0x03 gives ~44Hz bandwidth.
    uint8_t config_data[] = {0x1A, 0x03};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, config_data, 2, false);
    sleep_ms(10);

    // Set gyroscope full scale range to ±250 deg/s
    uint8_t gyro_config[2] = {0x1B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    // Set accelerometer full scale range to ±2g
    uint8_t accel_config[2] = {0x1C, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);
    sleep_ms(10);
}

//------------------------------------------------------------------------------
// Read raw sensor data from the MPU6050
//------------------------------------------------------------------------------
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t* temp) {
    uint8_t reg = 0x3B;  // Starting register: ACCEL_XOUT_H
    uint8_t buffer[6];

    // Read accelerometer data (6 bytes)
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Read gyroscope data (starting at GYRO_XOUT_H, register 0x43)
    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[2*i] << 8) | buffer[2*i+1];
    }

    // Read temperature data (2 bytes from TEMP_OUT_H, register 0x41)
    reg = 0x41;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 2, false);
    *temp = (buffer[0] << 8) | buffer[1];
}

//------------------------------------------------------------------------------
// Calibrate sensor offsets (run while the MPU6050 is stationary and level)
//------------------------------------------------------------------------------
void mpu6050_calibrate(float* accel_offset, float* gyro_offset, int num_samples) {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;

    // Allow time for the sensor to stabilize
    sleep_ms(1000);

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

    // Compute averages
    float ax_avg = (float)ax_sum / num_samples;
    float ay_avg = (float)ay_sum / num_samples;
    float az_avg = (float)az_sum / num_samples;
    float gx_avg = (float)gx_sum / num_samples;
    float gy_avg = (float)gy_sum / num_samples;
    float gz_avg = (float)gz_sum / num_samples;

    // For accelerometer: assume X and Y should be 0g and Z should read +1g
    accel_offset[0] = ax_avg;               // X-axis offset
    accel_offset[1] = ay_avg;               // Y-axis offset
    accel_offset[2] = az_avg - ACCEL_LSB_2G;  // Z-axis offset adjustment

    // For gyroscope: ideally 0 deg/s
    gyro_offset[0] = gx_avg;
    gyro_offset[1] = gy_avg;
    gyro_offset[2] = gz_avg;

    #ifdef DEBUG_mpu
    // For debugging purposes, force offsets to zero
    accel_offset[0] = 0;
    accel_offset[1] = 0;
    accel_offset[2] = 0;
    gyro_offset[0] = 0;
    gyro_offset[1] = 0;
    gyro_offset[2] = 0;
    #endif
}

//------------------------------------------------------------------------------
// Obtain offset-corrected sensor values with additional filtering to smooth out noise
//------------------------------------------------------------------------------
void mpu6050_adjusted_values(SensorData_t* data, float accel_offset[3], float gyro_offset[3]) {
    // Read raw sensor values
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);

    // Apply offset correction
    float ax_corr = (float)accel_raw[0] - accel_offset[0];
    float ay_corr = (float)accel_raw[1] - accel_offset[1];
    float az_corr = (float)accel_raw[2] - accel_offset[2];

    float gx_corr = (float)gyro_raw[0] - gyro_offset[0];
    float gy_corr = (float)gyro_raw[1] - gyro_offset[1];
    float gz_corr = (float)gyro_raw[2] - gyro_offset[2];

    // --- Exponential Moving Average Filtering ---
    // These static variables hold the previous filtered values.
    static int first_run = 1;
    static float filtered_ax, filtered_ay, filtered_az;
    static float filtered_gx, filtered_gy, filtered_gz;
    const float alpha = 0.2f;  // Smoothing factor (0 < alpha <= 1)

    if (first_run) {
        filtered_ax = ax_corr;
        filtered_ay = ay_corr;
        filtered_az = az_corr;
        filtered_gx = gx_corr;
        filtered_gy = gy_corr;
        filtered_gz = gz_corr;
        first_run = 0;
    } else {
        filtered_ax = alpha * ax_corr + (1.0f - alpha) * filtered_ax;
        filtered_ay = alpha * ay_corr + (1.0f - alpha) * filtered_ay;
        filtered_az = alpha * az_corr + (1.0f - alpha) * filtered_az;

        filtered_gx = alpha * gx_corr + (1.0f - alpha) * filtered_gx;
        filtered_gy = alpha * gy_corr + (1.0f - alpha) * filtered_gy;
        filtered_gz = alpha * gz_corr + (1.0f - alpha) * filtered_gz;
    }

    // Convert accelerometer values from LSB to g
    data->ax_g = filtered_ax / ACCEL_LSB_2G;
    data->ay_g = filtered_ay / ACCEL_LSB_2G;
    data->az_g = filtered_az / ACCEL_LSB_2G;

    // Convert gyroscope values from LSB to deg/s then to rad/s
    float gx_dps = filtered_gx / GYRO_LSB_250;
    float gy_dps = filtered_gy / GYRO_LSB_250;
    float gz_dps = filtered_gz / GYRO_LSB_250;

    data->gx_rad = gx_dps * (float)M_PI / 180.0f;
    data->gy_rad = gy_dps * (float)M_PI / 180.0f;
    data->gz_rad = gz_dps * (float)M_PI / 180.0f;

    #ifdef DEBUG_mpu
    data->ax_g = 0.0f;
    data->ay_g = 0.0f;
    data->az_g = 0.0f;
    data->gx_rad = 0.0f;
    data->gy_rad = 0.0f;
    data->gz_rad = 0.0f;
    #endif
}
