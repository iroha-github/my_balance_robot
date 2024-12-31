#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "mpu6050_i2c.h"       // <-- センサー操作のヘッダ
#include "madgwick_filter.h"
#include "pid_controller.h"
#include "config.h"

// --- 定数定義 ---
#define SERVO_PIN_RIGHT 26
#define SERVO_PIN_LEFT  27
// #define I2C_PORT i2c1
// #define I2C_SCL_PIN 10
// #define I2C_SDA_PIN 11

// 0SDA 4
// 0SCL 5
// 0SDA 12
// 0SCL 13
// 1SDA 14
// 1SCL 15

// サーボ制御パラメータ (50Hz, 20ms周期)
#define SERVO_FREQ_HZ   50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) // 20000us = 20ms

// 1500.0はdouble型／1500.0fはfloat型

// パルス幅(μs)の目安
//   1500μs 付近で停止, 2300μs で前進, 700μs で後退(一例)
#define SERVO_NEUTRAL_US 1500.0f
#define SERVO_RANGE_US    800.0f  // ±800μsを全速幅とする(例)

// PIDゲイン(要調整)
#define PID_KP  10.0f
#define PID_KI   0.0f
#define PID_KD   1.0f

// Madgwickフィルタゲイン
#define MADGWICK_BETA 0.05f // 元々は0.05fだった

// ±2g, ±250deg/sの場合
#define ACCEL_LSB_2G 16384.0f
#define GYRO_LSB_250 131.0f

// 関数にstaticを付けると、その関数はそのファイル内でのみ有効になる
static void init_pwm_for_servo(uint pin);
static void set_servo_pulse(uint pin, float pulse_us);

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
    init_pwm_for_servo(SERVO_PIN_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT);

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

        // 連続回転サーボ速度指令 (例: ±1.0にクリップ)
        float right_speed = pid_output;
        float left_speed  = -pid_output;

        if (right_speed > 1.0f)  right_speed = 1.0f;
        if (right_speed < -1.0f) right_speed = -1.0f;
        if (left_speed > 1.0f)   left_speed = 1.0f;
        if (left_speed < -1.0f)  left_speed = -1.0f;

        // (E) サーボパルス生成(1500±500us)
        float right_pulse = SERVO_NEUTRAL_US + SERVO_RANGE_US * right_speed;
        float left_pulse  = SERVO_NEUTRAL_US + SERVO_RANGE_US * left_speed;

        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // デバッグ出力
        // printf("---------------------------\n");
        // printf("pitch=%.2f [deg], pid=%.2f\n", pitch, pid_output);
        // printf("R_pulse=%.1f, L_pulse=%.1f\n", right_pulse, left_pulse);
        printf(">pitch:%.2f\n",pitch);
        printf(">pid:%.2f\n", pid_output);
        printf(">R_pulse:%.1f\n", right_pulse);
        printf(">L_pulse:%.1f\n", left_pulse);
        sleep_ms(20);
    }

    return 0;
}

//--------------------------------------------------
// サーボPWM初期化
//--------------------------------------------------
static void init_pwm_for_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM); //GPIO機能を選択している，GPIO_FUNC_PWMはPWM機能を選択していることになる

    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();
    // 例: wrap=24999, clkdiv=96 => 50Hz
    pwm_config_set_clkdiv(&config, 96.0f);
    pwm_config_set_wrap(&config, 24999);
    pwm_init(slice_num, &config, true);
    pwm_set_enabled(slice_num, true);

    // 初期はニュートラル
    set_servo_pulse(pin, SERVO_NEUTRAL_US);
}

//--------------------------------------------------
// サーボ用パルス幅(μs)設定
//--------------------------------------------------
static void set_servo_pulse(uint pin, float pulse_us) {
    if (pulse_us < 500.0f)  pulse_us = 500.0f;
    if (pulse_us > 2500.0f) pulse_us = 2500.0f;

    float duty_cycle = pulse_us / (float)SERVO_PERIOD_US; // 20ms周期での比率
    float compare    = duty_cycle * 25000.0f;             // wrap=24999なら0~25000

    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);

    pwm_set_chan_level(slice_num, channel, (uint16_t)compare);
}
