// src/main.c (変更例)

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// Madgwick, PID などの既存ヘッダ
#include "madgwick_filter.h"
#include "mpu6050_i2c.h"
#include "pid_controller.h"
#include "servo.h"
#include "onboard_led.h"
#include "config.h"

// 距離センサ(HC-SR04)関連
#include "hcsr04.h"

// ロータリスイッチ関連(省略可能・必要に応じて)
#include "rotary_switch.h"

#define DEBUG  // ★DEBUGモードを有効化

// グローバル変数(オフセット)
float accel_offset_global[3] = {0};
float gyro_offset_global[3]  = {0};

int main() {
    stdio_init_all();
    pico_led_init();
    pico_set_led(true);
    sleep_ms(500);

    printf("倒立振子 + HC-SR04 デバッグテスト\n");

    pico_set_led(false);

    // I2C初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // HC-SR04初期化
    init_hcsr04();

    // ロータリスイッチ初期化(例)
    init_rotary_switch();

    // MPU6050リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 200);
    printf("キャリブレーション完了\n");

    // Madgwickフィルタ初期化
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);

    // PID制御初期化
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);

    // サーボPWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT,  SERVO_NEUTRAL_LEFT);

    absolute_time_t prev_time = get_absolute_time();
    float pitch_target = 0.0f; 

    pico_set_led(true);

    while (1) {
        // (A) センサ取得
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);

        // (B) Δt計算
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;

        // (C) Madgwick更新
        MadgwickAHRSupdateIMU(&madgwick,
                              sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
                              sensor_data.ax_g,   sensor_data.ay_g,   sensor_data.az_g,
                              dt);
        float roll_deg, pitch_deg, yaw_deg;
        MadgwickGetEulerDeg(&madgwick, &roll_deg, &pitch_deg, &yaw_deg);

        // (D) PID
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch_deg, dt);

        // (E) サーボ
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // ──────────────────────────────────────────
        // デバッグ出力(距離＋他情報を連続表示)
        // ──────────────────────────────────────────
#ifdef DEBUG
        float distance = measure_distance_cm(); // 距離取得
        printf(">pitch: %.2f deg\n", pitch_deg);
        printf(">pid_output: %.2f\n", pid_output);
        printf(">Right_pulse: %.1f\n", right_pulse);
        printf(">Left_pulse: %.1f\n", left_pulse);
        printf(">distance: %.2f cm\n", distance);  // 新規追加
        sleep_ms(20);
#else
        sleep_ms(20);
#endif
    }

    return 0;
}
