// src/main.c

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

// 既存ヘッダ類
#include "madgwick_filter.h"
#include "mpu6050_i2c.h"
#include "pid_controller.h"
#include "servo.h"
#include "onboard_led.h"
#include "config.h"

// ★ ロータリスイッチとHC-SR04
#include "rotary_switch.h"
#include "hcsr04.h"

// ★★★ 追加 ★★★ フルカラーLED制御ヘッダ
#include "fullcolor_led.h"

#define DEBUG  // ★DEBUG出力を有効にする

// グローバルオフセット（キャリブレーション用）
float accel_offset_global[3] = {0};
float gyro_offset_global[3]  = {0};

int main() {
    stdio_init_all();

    // (1) オンボードLEDの初期化と点灯テスト
    pico_led_init();
    pico_set_led(true);
    sleep_ms(500);
    pico_set_led(false);

    // (2) ★ フルカラーLEDの初期化
    fullcolor_led_init();
    // オフにする場合： set_fullcolor_led_rgb(0, 0, 0);

    printf("倒立振子 + HC-SR04 + ロータリスイッチ + フルカラーLED テスト\n");

    // (3) I2C (MPU6050) 初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // (4) 超音波センサ HC-SR04 初期化
    init_hcsr04();

    // (5) ロータリスイッチ 初期化
    init_rotary_switch();

    // (6) MPU6050 リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 200);
    printf("キャリブレーション完了\n");

    // (7) Madgwick フィルタの初期化
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);

    // (8) PID コントローラ初期化 (ピッチ制御用)
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);

    // (9) サーボPWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT,  SERVO_NEUTRAL_LEFT);

    // (10) 倒立振子の目標ピッチ角
    float pitch_target = 0.0f;

    // 時間計測用
    absolute_time_t prev_time = get_absolute_time();

    // 一度オンボードLEDを点灯
    pico_set_led(true);

    while (1) {
        // 1) MPU6050 取得
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);

        // 2) Δt計算
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;

        // 3) Madgwick更新 (姿勢推定)
        MadgwickAHRSupdateIMU(
            &madgwick,
            sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
            sensor_data.ax_g,   sensor_data.ay_g,   sensor_data.az_g,
            dt
        );
        float roll_deg, pitch_deg, yaw_deg;
        MadgwickGetEulerDeg(&madgwick, &roll_deg, &pitch_deg, &yaw_deg);

        // 4) PID 制御 (ターゲットは pitch=0.0度)
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch_deg, dt);

        // 5) サーボ出力計算 & セット
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // 6) ロータリスイッチ状態を読み取り
        int mode = read_rotary_switch_mode();

        // 7) 距離センサ (mode=0 の時だけ計測例)
        float distance = -999.0f;
        if (mode == 0) {
            distance = measure_distance_cm();
        }

        // 8) ★ フルカラーLEDの色をモードに応じて変える例 ★
        switch (mode) {
            case 0: // 赤色
                set_fullcolor_led_rgb(255, 0, 0);
                break;
            case 1: // 緑色
                set_fullcolor_led_rgb(0, 255, 0);
                break;
            case 2: // 青色
                set_fullcolor_led_rgb(0, 0, 255);
                break;
            case 3: // 白色
                set_fullcolor_led_rgb(255, 255, 255);
                break;
            default:
                // エラーまたは複数High => 消灯
                set_fullcolor_led_rgb(0, 0, 0);
                break;
        }

#ifdef DEBUG
        // === デバッグ出力 ===
        switch (mode) {
            case 0:
                printf("[Rotary] Switch=0 => 距離センサ使用\n");
                break;
            case 1:
                printf("[Rotary] Switch=1\n");
                break;
            case 2:
                printf("[Rotary] Switch=2\n");
                break;
            case 3:
                printf("[Rotary] Switch=3\n");
                break;
            default:
                printf("[Rotary] Switch=-1 (エラー)\n");
                break;
        }

        printf(">pitch: %.2f deg\n", pitch_deg);
        printf(">pid_output: %.2f\n", pid_output);
        printf(">Right_pulse: %.1f\n", right_pulse);
        printf(">Left_pulse: %.1f\n", left_pulse);

        if (mode == 0) {
            if (distance >= 0.0f) {
                printf("> distance: %.2f cm\n", distance);
            } else {
                printf("> distance: Error=%.1f\n", distance);
            }
        }

        printf("----------------\n");
        sleep_ms(20); // デバッグ用
#else
        // デバッグしない場合でも周期を整えるために少し待つ
        sleep_ms(20);
#endif
    }

    return 0;
}
