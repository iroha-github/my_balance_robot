// config.h
#ifndef CONFIG_H
#define CONFIG_H

// --- 定数定義 ---
#define SERVO_PIN_RIGHT 26
#define SERVO_PIN_LEFT  27

#define I2C_PORT i2c1
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 10

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


#endif // CONFIG_H
