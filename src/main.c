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

#define DEBUG  // ★DEBUG出力を有効にする

// グローバルオフセット
float accel_offset_global[3] = {0};
float gyro_offset_global[3]  = {0};

int main() {
    stdio_init_all();
    pico_led_init();
    pico_set_led(true);
    sleep_ms(500);

    printf("倒立振子 + HC-SR04 + ロータリスイッチ テスト\n");
    pico_set_led(false);

    // I2C(MPU6050)初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // HC-SR04初期化
    init_hcsr04();

    // ロータリスイッチ初期化
    init_rotary_switch();

    // MPU6050 リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 200);
    printf("キャリブレーション完了\n");

    // Madgwickフィルタ
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);

    // PID
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);

    // サーボPWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT,  SERVO_NEUTRAL_LEFT);

    float pitch_target = 0.0f;
    absolute_time_t prev_time = get_absolute_time();

    pico_set_led(true);

    while (1) {
        // 1) MPU6050 取得
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);

        // 2) Δt
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;

        // 3) Madgwick更新
        MadgwickAHRSupdateIMU(
            &madgwick,
            sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
            sensor_data.ax_g,   sensor_data.ay_g,   sensor_data.az_g,
            dt
        );
        float roll_deg, pitch_deg, yaw_deg;
        MadgwickGetEulerDeg(&madgwick, &roll_deg, &pitch_deg, &yaw_deg);

        // 4) PID
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch_deg, dt);

        // 5) サーボ出力計算
        float right_pulse, left_pulse;
        calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // 6) ロータリスイッチ状態
        int mode = read_rotary_switch_mode();

        // 7) 距離センサを使うかなどの処理
        //    例: mode=0 の時だけ距離測定
        float distance = -999.0f; // デフォルト(未使用)
        if (mode == 0) {
            distance = measure_distance_cm();
            // 必要に応じて距離補正やほかの処理
        }

#ifdef DEBUG
        // === デバッグ出力 ===
        // どのスイッチモードか表示
        switch (mode) {
            case 0:
                printf("[Rotary] Switch=0 => 距離センサ使用\n");
                break;
            case 1:
                printf("[Rotary] Switch=1\n");
                break;
            case 2:
                printf("[Rotary] Switch=2 => ？自動自立モード\n");
                break;
            case 3:
                printf("[Rotary] Switch=3 => ？bluetoorh\n");
                break;
            default:
                printf("[Rotary] Switch=-1 (エラー or 複数HIGH)\n");
                break;
        }

        // 姿勢やPIDのデバッグ表示
        printf(">pitch: %.2f deg\n", pitch_deg);
        printf(">pid_output: %.2f\n", pid_output);
        printf(">Right_pulse: %.1f\n", right_pulse);
        printf(">Left_pulse: %.1f\n", left_pulse);

        // 距離センサ
        if (mode == 0) {
            // mode=0のときのみ計測結果を出力
            if (distance >= 0.0f) {
                printf(">distance: %.2f cm\n", distance);
            } else {
                // -1 or -2 ならタイムアウトなど
                printf(">distance: Error=%.1f\n", distance);
            }
        }

        printf("----------------\n");
        sleep_ms(20);  // デバッグ用途であれば少し長めに待つ
#else
        // DEBUGでなければループをある程度の周期で回す(例:20ms)
        sleep_ms(20);
#endif
    }

    return 0;
}
