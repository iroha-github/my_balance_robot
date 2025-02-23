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
#include "rotary_switch.h" 
#include "hcsr04.h"
#include "fullcolor_led.h"
#include "command_input.h" // コマンド入力用

#define DEBUG  // ★DEBUG出力を有効にする

// グローバルオフセット（キャリブレーション用）
float accel_offset_global[3] = {0};
float gyro_offset_global[3]  = {0};

int main() {
    stdio_init_all();

    printf("倒立振子 + HC-SR04 + ロータリスイッチ + フルカラーLED + シリアル入力テスト\n");

    // (1) オンボードLEDの初期化と動作テスト
    onboard_led_init();
    onboard_led_set(true);

    // (2) I2C (MPU6050) 初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // (3) 各種初期化
    fullcolor_led_init(); // フルカラーLEDの初期化
    hcsr04_init(); // HC-SR04の初期化
    rotary_switch_init(); // ロータリスイッチの初期化

    sleep_ms(500);
    onboard_led_set(false);

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

    // (10) 倒立振子の目標ピッチ角（基本は直立＝0.0 deg）
    float base_pitch_target = 0.0f;

    // シリアル入力による動作指令を保持する変数
    CommandInput_t cmd = {0.0f, 0.0f};
    init_command_input();

    // 時間計測用
    absolute_time_t prev_time = get_absolute_time();

    // 一度オンボードLEDを点灯
    onboard_led_set(true);

    while (1) {
        // ★★★ ① シリアル入力からコマンドを更新
        update_command_input(&cmd);

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

        // ★★★ ② シリアル入力で得た前後指令を目標ピッチに加える
        float effective_pitch_target = base_pitch_target + cmd.pitch_offset;

        // 4) PID 制御 (ターゲットは effective_pitch_target)
        float pid_output = PID_Update(&pid_pitch, effective_pitch_target, pitch_deg, dt);

        // 5) サーボ出力計算
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);

        // ★★★ ③ シリアル入力で得た左右旋回指令をサーボパルスに加える
        // 右側に turn_offset、左側に -turn_offset を加える（※各サーボの範囲内に収める必要あり）
        right_pulse += cmd.turn_offset;
        left_pulse  -= cmd.turn_offset;
        // ※ 必要なら、SERVO_MIN/MAX を用いて clamp（制限）してください

        // サーボ出力設定
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // 6) ロータリスイッチ状態を読み取り（従来通り）
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

        printf(">pitch[deg]: %.2f\n",pitch_deg);
        printf(">effective_target[deg]: %.2f\n",effective_pitch_target);
        printf(">pid_output: %.2f\n", pid_output);
        printf(">Right_pulse: %.1f\n", right_pulse);
        printf(">Left_pulse: %.1f\n",  left_pulse);

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
