// src/main.c
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "madgwick_filter.h"
#include "mpu6050_i2c.h"
#include "pid_controller.h"
#include "servo.h"
#include "config.h"
#include "ultrasonic.h"

// デバッグ出力(必要に応じてコメントアウト)
#define DEBUG

// モード定義
typedef enum {
    MODE_A = 0,
    MODE_B = 1,
    MODE_C = 2,
    MODE_D = 3,
} RobotMode_t;

// グローバル変数としてオフセットを定義
static float accel_offset_global[3] = {0};
static float gyro_offset_global[3]  = {0};

//------------------------------------------------------------------------------
// ロータリースイッチの状態を読み取り、モードを決定する
// (ここでは単純に各ピンが0になっているかどうかで判定する例)
//------------------------------------------------------------------------------
static RobotMode_t read_mode_from_rotary_switch(void) {
    // ロータリースイッチの4端子を入力に設定
    gpio_init(ROTARY_PIN_A);
    gpio_set_dir(ROTARY_PIN_A, GPIO_IN);
    // 内部プルダウンを有効にする (環境に応じて外部プルダウンの場合は不要)
    gpio_pull_down(ROTARY_PIN_A);

    gpio_init(ROTARY_PIN_B);
    gpio_set_dir(ROTARY_PIN_B, GPIO_IN);
    gpio_pull_down(ROTARY_PIN_B);

    gpio_init(ROTARY_PIN_C);
    gpio_set_dir(ROTARY_PIN_C, GPIO_IN);
    gpio_pull_down(ROTARY_PIN_C);

    gpio_init(ROTARY_PIN_D);
    gpio_set_dir(ROTARY_PIN_D, GPIO_IN);
    gpio_pull_down(ROTARY_PIN_D);

    // 立ち上がり待ち
    sleep_ms(50);

    // COM が 3.3V に接続され、選択されたピンが High(=1) になる想定
    bool a_state = (gpio_get(ROTARY_PIN_A) == 1);
    bool b_state = (gpio_get(ROTARY_PIN_B) == 1);
    bool c_state = (gpio_get(ROTARY_PIN_C) == 1);
    bool d_state = (gpio_get(ROTARY_PIN_D) == 1);

    // どのピンが 1 になっているかでモードを判定
    if (a_state) return MODE_A;
    if (b_state) return MODE_B;
    if (c_state) return MODE_C;
    if (d_state) return MODE_D;

    // 全部0の場合はデフォルトで MODE_A にしておく
    return MODE_A;
}


//------------------------------------------------------------------------------
// LED初期化 & 指定色点灯
//------------------------------------------------------------------------------
static void set_leds_for_mode(RobotMode_t mode) {
    gpio_init(RED_LED_PIN);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);

    gpio_init(BLUE_LED_PIN);
    gpio_set_dir(BLUE_LED_PIN, GPIO_OUT);

    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

    // いったん全消灯
    gpio_put(RED_LED_PIN,   0);
    gpio_put(BLUE_LED_PIN,  0);
    gpio_put(GREEN_LED_PIN, 0);

    // モードに応じて点灯
    switch (mode) {
        case MODE_A:
            gpio_put(BLUE_LED_PIN, 1);
            break;
        case MODE_B:
            gpio_put(GREEN_LED_PIN, 1);
            break;
        case MODE_C:
            gpio_put(RED_LED_PIN, 1);
            break;
        case MODE_D:
            gpio_put(RED_LED_PIN,   1);
            gpio_put(BLUE_LED_PIN,  1);
            gpio_put(GREEN_LED_PIN, 1);
            break;
        default:
            break;
    }
}

// メイン関数
int main() {
    stdio_init_all();

    // LED点灯
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);


    sleep_ms(500); // シリアル出力待ち

    printf("倒立振子 + PID + Madgwick + 距離センサ + モード制御\n");

    // モード判定
    RobotMode_t mode = read_mode_from_rotary_switch();
    printf("選択されたモード: %d\n", mode);

    
    // I2C初期化(MPU6050)
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // MPU6050リセット & キャリブレーション
    mpu6050_reset();
    printf("MPU6050キャリブレーション中...\n");
    mpu6050_calibrate(accel_offset_global, gyro_offset_global, 500);
    printf("キャリブレーション完了!\n");
    
    // Madgwickフィルタ初期化
    MadgwickFilter_t madgwick;
    MadgwickInit(&madgwick, MADGWICK_BETA);
    
    // PID初期化
    PID_t pid_pitch;
    PID_Init(&pid_pitch, PID_KP, PID_KI, PID_KD);
    
    // サーボPWM初期化
    init_pwm_for_servo(SERVO_PIN_RIGHT, SERVO_NEUTRAL_RIGHT);
    init_pwm_for_servo(SERVO_PIN_LEFT,  SERVO_NEUTRAL_LEFT);
    
    // 距離センサ
    bool use_ultrasonic = false;
    if (mode == MODE_B || mode == MODE_C) {
        // BとCでは距離センサON
        ultrasonic_init();
        use_ultrasonic = true;
        printf("距離センサ使用\n");
    }
    
    // 目標ピッチ(倒立)を0度とする
    float pitch_target = 0.0f;
    absolute_time_t prev_time = get_absolute_time();
    
    // MODE_C用の簡易フラグ(回転中かどうか)
    bool turning = false;

    // LED設定
    set_leds_for_mode(mode);
    
    while (1) {
        // (1) センサ読み取り
        SensorData_t sensor_data;
        mpu6050_adjusted_values(&sensor_data, accel_offset_global, gyro_offset_global);
        
        // Δt
        absolute_time_t now = get_absolute_time();
        float dt = (float)(to_us_since_boot(now) - to_us_since_boot(prev_time)) / 1e6f;
        prev_time = now;
        if (dt <= 0.0f) dt = 0.001f;

        // (2) 姿勢推定(Madgwick)
        MadgwickAHRSupdateIMU(
            &madgwick,
            sensor_data.gx_rad, sensor_data.gy_rad, sensor_data.gz_rad,
            sensor_data.ax_g,  sensor_data.ay_g,    sensor_data.az_g,
            dt
        );
        float roll, pitch, yaw;
        MadgwickGetEulerDeg(&madgwick, &roll, &pitch, &yaw);

        // (3) PID計算
        float pid_output = 0.0f;
        if (mode != MODE_D) {
            // MODE_D は常にサーボをニュートラルにするためPID計算しない
            pid_output = PID_Update(&pid_pitch, pitch_target, pitch, dt);
        }

        // (4) 超音波センサの値を取得 (必要なモードのみ)
        float distance_cm = -1.0f;
        if (use_ultrasonic) {
            distance_cm = get_distance_cm(); 
            // -1.0f はタイムアウトなど(適宜処理)
        }

        // (5) モード別動作
        float right_pulse = SERVO_NEUTRAL_RIGHT;
        float left_pulse  = SERVO_NEUTRAL_LEFT;

        switch (mode) {
            case MODE_A:
                // A: 青色LED点灯、普通のPID制御、距離センサオフ
                //   → そのままPID出力を使用し、サーボ駆動
                calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
                break;

            case MODE_B:
                // B: 緑色LED点灯、距離センサON
                //    距離が15cm以下になったら後方へ移動(姿勢はPIDで保つ)
                if (distance_cm > 0 && distance_cm <= 10.0f) {
                    // 後進させたいのでPID出力に加算/減算で実現
                    // 例: さらに後進量を固定値で足す
                    float backward_factor = -0.3f; // 固定で少し後ろへ
                    calculate_servo_pulse(pid_output + backward_factor, &right_pulse, &left_pulse);
                    printf(">Backward:%d\n",0);
                } else {
                    // 通常どおりPIDのみ
                    calculate_servo_pulse(pid_output, &right_pulse, &left_pulse);
                    printf(">Backward:%d\n",1);
                }
                break;

            case MODE_C:
                // C: 赤色LED点灯、距離センサON
                //    前進しながら、距離が10cm以下なら左右どちらかに回転し続ける
                //    再び10cmを超えたら前進に戻る
                if (distance_cm > 0 && distance_cm <= 10.0f) {
                    // 10cm以下 → 回転動作に入る
                    turning = true;
                } else if (distance_cm > 10.0f) {
                    // 10cm超えたら前進モードへ
                    turning = false;
                }

                if (turning) {
                    // 左右どちらかに回転 (例: 左回転)
                    float turn_factor = 0.3f;
                    // PID出力に加算して左右差を大きくする or 
                    // 直接一方を前進/もう一方を後退させる etc.
                    // ここでは簡単にPID出力+turn_factorで実装
                    calculate_servo_pulse(pid_output + turn_factor, &right_pulse, &left_pulse);
                } else {
                    // 前進 + PID
                    float forward_factor = 0.3f;
                    calculate_servo_pulse(pid_output + forward_factor, &right_pulse, &left_pulse);
                }
                break;

            case MODE_D:
                // D: 全LED点灯, 距離センサオフ
                //    サーボをニュートラルに固定
                right_pulse = SERVO_NEUTRAL_RIGHT;
                left_pulse  = SERVO_NEUTRAL_LEFT;
                
                break;

            default:
                // 想定外の場合、停止にしておく
                right_pulse = SERVO_NEUTRAL_RIGHT;
                left_pulse  = SERVO_NEUTRAL_LEFT;
                break;
        }

        // (6) サーボ出力
        set_servo_pulse(SERVO_PIN_RIGHT, right_pulse);
        set_servo_pulse(SERVO_PIN_LEFT,  left_pulse);

        // (7) デバッグ出力
        #ifdef DEBUG
        printf(">Mode:%d\n",mode);
        printf(">pitch:%.2f\n",pitch);
        printf(">pid:%.2f\n",pid_output);
        printf(">dist:%.1f\n",distance_cm);
        printf(">R:%.1f\n",right_pulse);
        printf(">L:%.1f\n",left_pulse);

        #endif

        sleep_ms(20); // 50Hz相当
    }

    return 0;
}
