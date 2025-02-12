// src/main.c
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "madgwick_filter.h"
#include "mpu6050_i2c.h"       // センサー操作のヘッダ
#include "pid_controller.h"
#include "servo.h"              // サーボ操作のヘッダ
#include "config.h"

// デバッグ出力
#define DEBUG

// グローバル変数としてオフセットを定義
float accel_offset_global[3] = {0};
float gyro_offset_global[3] = {0};

int main() {
    stdio_init_all();
    sleep_ms(500); // シリアル出力待ち等(短め)

    printf("倒立振子 + PID + Madgwick テスト開始\n");

    // I2C初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // 1) MPU6050リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中。水平に置いて静止してください...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 500); // グローバル変数を使用
    printf("キャリブレーション完了!\n");

    // 2) Madgwickフィルタ初期化
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);

    // 3) PID制御初期化
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);

    // 4) サーボ用PWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT,  SERVO_NEUTRAL_LEFT);

    absolute_time_t prev_time = get_absolute_time();
    float pitch_target = 0.0f; // 直立を0度とする

    while (1) {
        // (A) MPU6050 オフセット後のデータ取得
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);

        // (B) Δt
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;

        // (C) Madgwick更新
        MadgwickAHRSupdateIMU(&madgwick, sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
                                sensor_data.ax_g, sensor_data.ay_g, sensor_data.az_g, dt);
        float roll, pitch, yaw;
        MadgwickGetEulerDeg(&madgwick, &roll, &pitch, &yaw);

        // (D) PID計算 (目標ピッチ0度)
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch, dt);

        // (E) サーボパルス計算
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);

        // (F) サーボパルス生成output
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // デバッグ出力
        #ifdef DEBUG
            printf(">pitch:%.2f\n", pitch);
            printf(">pid_output:%.2f\n", pid_output);
            printf(">Right_pulse:%.1f\n", right_pulse);
            printf(">Left_pulse:%.1f\n", left_pulse);
            sleep_ms(20); // 50Hzという意味...？
        #endif
    }

    return 0;
}
