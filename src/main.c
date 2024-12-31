#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "madgwick_filter.h"
#include "mpu6050_i2c.h"       // <-- センサー操作のヘッダ
#include "pid_controller.h"
#include "servo.h"              // <-- サーボ操作のヘッダ
#include "config.h"

// デバッグ出力
#define DEBUG


// PIDゲイン(要調整)
#define PID_KP  10.0f
#define PID_KI   0.0f
#define PID_KD   1.0f

// グローバル変数としてオフセットを定義
float accel_offset_global[3] = {0};
float gyro_offset_global[3] = {0};

int main() {
    stdio_init_all();
    sleep_ms(2000); // シリアル出力待ち等

    printf("倒立振子 + PID + Madgwick テスト開始\n");

    // I2C初期化
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // 1) MPU6050リセット & キャリブレーション
    mpu6050_reset();
    float accel_offset[3] = {0}, gyro_offset[3] = {0};
    printf("MPU6050キャリブレーション中。水平に置いて静止してください...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 200); // サンプル数200は一例
    printf("キャリブ完了!\n");

    // 2) Madgwickフィルタ初期化
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);

    // 3) PID制御初期化
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);

    // 4) サーボ用PWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT, SERVO_NEUTRAL_LEFT);

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
        MadgwickAHRSupdateIMU(&madgwick, sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad, sensor_data.ax_g, sensor_data.ay_g, sensor_data.az_g, dt);
        float roll, pitch, yaw;
        MadgwickGetEulerDeg(&madgwick, &roll, &pitch, &yaw);

        // (D) PID計算 (目標ピッチ0度)
        float pid_output = PID_Update(&pid_pitch, pitch_target, pitch, dt);

        // (E) サーボパルス受け渡し
        float right_speed = pid_output;
        float left_speed  = -pid_output;

        // (F) サーボパルス計算
        float right_pulse, left_pulse;
        calculate_servo_pulse(right_speed, left_speed, &right_pulse, &left_pulse);

        // (G) サーボパルス生成
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // デバッグ出力
        #ifdef DEBUG
            printf(">pitch:%.2f\n",pitch);
            printf(">pid:%.2f\n", pid_output);
            printf(">R_pulse:%.1f\n", right_pulse);
            printf(">L_pulse:%.1f\n", left_pulse);
            sleep_ms(20); // 50Hzという意味...？
        #endif
    }

    return 0;
}
