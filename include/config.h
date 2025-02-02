// config.h
#ifndef CONFIG_H
#define CONFIG_H

// ---------------------------------------------------------
// 既存の定義
// ---------------------------------------------------------
#define PID_KP  0.5f
#define PID_KI  0.1f
#define PID_KD  0.0f

#define SERVO_PIN_RIGHT 6
#define SERVO_PIN_LEFT  7

#define I2C_PORT i2c0
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13

#define MADGWICK_BETA 0.05f

#define SERVO_NEUTRAL_RIGHT 1500.0f
#define SERVO_NEUTRAL_LEFT  1500.0f
#define SERVO_RANGE_RIGHT    800.0f
#define SERVO_RANGE_LEFT     800.0f

#define SERVO_FREQ_HZ   50
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ)
#define SERVO_CORRECTION 30.0f

// ---------------------------------------------------------
// ★★★ 追加定義 ★★★ 
// HC-SR04ピン (Trig, Echo)
#define ULTRA_TRIG_PIN 10
#define ULTRA_ECHO_PIN 11

// ロータリスイッチ ピン (例: 18,19,20,21)
#define ROTARY_SW_PIN1 18
#define ROTARY_SW_PIN2 19
#define ROTARY_SW_PIN3 20
#define ROTARY_SW_PIN4 21

#endif // CONFIG_H
