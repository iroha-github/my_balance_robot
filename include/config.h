// config.h
#ifndef CONFIG_H
#define CONFIG_H

// PIDゲイン(要調整)
#define PID_KP  0.5f
#define PID_KI   0.1f
#define PID_KD   0.0f


// --- 定数定義 ---
#define SERVO_PIN_RIGHT 7
#define SERVO_PIN_LEFT  6

#define I2C_PORT i2c0
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

// 0SDA 4
// 0SCL 5
// 0SDA 12
// 0SCL 13
// 1SDA 14
// 1SCL 15

// Madgwickフィルタゲイン
#define MADGWICK_BETA 0.05f // 元々は0.05fだった

// 1500.0はdouble型／1500.0fはfloat型

// パルス幅(μs)の目安
//   1500μs 付近で停止, 2300μs で前進, 700μs で後退(一例)
#define SERVO_NEUTRAL_RIGHT 1500.0f
#define SERVO_NEUTRAL_LEFT  1500.0f
#define SERVO_RANGE_RIGHT    800.0f  // ±800μsを全速幅とする(例)
#define SERVO_RANGE_LEFT     800.0f  // ±800μsを全速幅とする(例)

// サーボ制御パラメータ (50Hz, 20ms周期)
#define SERVO_FREQ_HZ   50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) // 20000us = 20ms

//　サーボ補正値
#define SERVO_CORRECTION 30.0f

#endif // CONFIG_H
