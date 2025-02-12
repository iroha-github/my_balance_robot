// config.h
#ifndef CONFIG_H
#define CONFIG_H

// PIDゲイン(要調整)
#define PID_KP  0.6f
#define PID_KI  0.2f
#define PID_KD  0.0f

// --- 定数定義 ---
#define SERVO_PIN_RIGHT 7
#define SERVO_PIN_LEFT  6

#define I2C_PORT i2c0
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

// Madgwickフィルタゲイン
#define MADGWICK_BETA 0.05f

// パルス幅(μs)の目安
//   1500μs 付近で停止, 2300μs で前進, 700μs  で後退(一例)
#define SERVO_NEUTRAL_RIGHT 1500.0f
#define SERVO_NEUTRAL_LEFT  1500.0f
#define SERVO_RANGE_RIGHT    800.0f
#define SERVO_RANGE_LEFT     800.0f

// サーボ制御パラメータ (50Hz, 20ms周期)
#define SERVO_FREQ_HZ   50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ)

// サーボ補正値
#define SERVO_CORRECTION 30.0f

// 距離センサ(HC-SR04)ピン
#define ULTRA_TRIG_PIN 16
#define ULTRA_ECHO_PIN 17

// LEDピン
#define BLUE_LED_PIN 2
#define GREEN_LED_PIN 3
#define RED_LED_PIN   4

// ロータリースイッチ用ピン (モード選択)
#define ROTARY_PIN_A 18
#define ROTARY_PIN_B 19
#define ROTARY_PIN_C 20
#define ROTARY_PIN_D 21

#endif // CONFIG_H
