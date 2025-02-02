// src/main.c
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
// #include "hardware/gpio.h"  // (不要になれば削除可)

#include "madgwick_filter.h"
#include "onboard_led.h"
#include "mpu6050_i2c.h"
#include "pid_controller.h"
#include "servo.h"
#include "config.h"

// ★追加: HC-SR04距離センサ関連
#include "hcsr04.h"

// ★追加: ロータリスイッチ関連
#include "rotary_switch.h"

// #define DEBUG // デバッグ出力を有効にする場合

// グローバル変数としてオフセットを定義
float accel_offset_global[3] = {0};
float gyro_offset_global[3]  = {0};

int main() {
    stdio_init_all();
    pico_led_init(); // LED初期化
    pico_set_led(true);
    sleep_ms(500);

    printf("倒立振子 + PID + Madgwick + HC-SR04 + RotarySwitch テスト開始\n");
    pico_set_led(false);

    // I2C初期化(MPU6050用)
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // ★追加: HC-SR04用初期化
    init_hcsr04();

    // ★追加: ロータリスイッチ用初期化
    init_rotary_switch();

    // 1) MPU6050リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中。水平に置いて静止してください...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 200);
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
    float pitch_target = 0.0f; // 直立を0度

    pico_set_led(true);

    while (1) {
        // (A) MPU6050 オフセット後のデータ取得
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);

        // (B) Δt
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;

        // (C) Madgwick更新
        MadgwickAHRSupdateIMU(&madgwick,
                              sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
                              sensor_data.ax_g,  sensor_data.ay_g,    sensor_data.az_g,
                              dt);
        float roll_deg, pitch_deg, yaw_deg;
        MadgwickGetEulerDeg(&madgwick, &roll_deg, &pitch_deg, &yaw_deg);

        // (D) PID計算
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch_deg, dt);

        // (E) サーボパルス計算 & 出力
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // --- ロータリスイッチのモード読取 ---
        int mode = read_rotary_switch_mode();

        // 距離センサを使用するかの判定
        if (mode == 0) {
            // GPIO18がHIGH => 距離センサ使用
            float dist_raw_cm = measure_distance_cm();
            if (dist_raw_cm > 0) {
                // ピッチ角で簡易補正 (例: dist*cos(pitch_rad))
                float pitch_rad = pitch_deg * (M_PI / 180.0f);
                float dist_corrected = dist_raw_cm * cosf(pitch_rad);

                printf("[Mode=0] Distance(Raw)=%.2f cm, Dist(Corrected)=%.2f cm, Pitch=%.2f deg\n",
                       dist_raw_cm, dist_corrected, pitch_deg);
            } else {
                // エラー
                printf("[Mode=0] Distance measurement error (raw=%.2f)\n", dist_raw_cm);
            }
        } else {
            // GPIO19,20,21がHIGHのいずれか or -1(エラー)
            // => 距離センサは使わない動作(例示)
            if (mode == 1) {
                // モード1の何か動作
            } else if (mode == 2) {
                // モード2の何か動作
            } else if (mode == 3) {
                // モード3の何か動作
            } else {
                // -1 => 複数/どれもHIGHでない などエラー
            }
        }

#ifdef DEBUG
        // デバッグ出力(任意)
        printf(">pitch:%.2f deg\n", pitch_deg);
        printf(">pid_output:%.2f\n", pid_output);
        printf(">Right_pulse:%.1f\n", right_pulse);
        printf(">Left_pulse:%.1f\n", left_pulse);
        sleep_ms(20);
#else
        sleep_ms(20);
#endif
    }

    return 0;
}
