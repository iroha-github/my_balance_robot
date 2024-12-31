// config.h
#ifndef CONFIG_H
#define CONFIG_H

#define I2C_PORT i2c1
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 10

// 1500.0はdouble型／1500.0fはfloat型

// パルス幅(μs)の目安
//   1500μs 付近で停止, 2300μs で前進, 700μs で後退(一例)
#define SERVO_NEUTRAL_US 1500.0f
#define SERVO_RANGE_US    800.0f  // ±800μsを全速幅とする(例)
#define SERVO_PERIOD_US


// サーボ制御パラメータ (50Hz, 20ms周期)
#define SERVO_FREQ_HZ   50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) // 20000us = 20ms


#endif // CONFIG_H
