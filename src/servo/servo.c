// src/servo/servo.c
#include <stdio.h>
#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "config.h" // SERVO_NEUTRAL_US, SERVO_RANGE_US, SERVO_PERIOD_US など

// static void set_servo_pulse(uint pin, float pulse_us);

// サーボの最大・最小パルス幅
#define SERVO_MIN_RIGHT (SERVO_NEUTRAL_RIGHT - SERVO_RANGE_RIGHT)
#define SERVO_MAX_RIGHT (SERVO_NEUTRAL_RIGHT + SERVO_RANGE_RIGHT)
#define SERVO_MIN_LEFT  (SERVO_NEUTRAL_LEFT  - SERVO_RANGE_LEFT)
#define SERVO_MAX_LEFT  (SERVO_NEUTRAL_LEFT  + SERVO_RANGE_LEFT)


void init_pwm_for_servo(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 96.0f); // 例: wrap=24999, clkdiv=96 => 50Hz
    pwm_config_set_wrap(&config, 24999);
    pwm_init(slice_num, &config, true);
    pwm_set_enabled(slice_num, true);
    
    // 初期はニュートラル
    set_servo_pulse(pin, SERVO_NEUTRAL_RIGHT);
    // set_servo_pulse(SERVO_PIN_LEFT, SERVO_NEUTRAL_LEFT);
}


void set_servo_pulse(uint pin, float pulse_us) {
    if (pulse_us < 500.0f)  pulse_us = 500.0f;
    if (pulse_us > 2500.0f) pulse_us = 2500.0f;
    
    float duty_cycle = pulse_us / (float)SERVO_PERIOD_US; // 20ms周期での比率
    float compare    = duty_cycle * 25000.0f;             // wrap=24999なら0~25000
    
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel   = pwm_gpio_to_channel(pin);
    
    pwm_set_chan_level(slice_num, channel, (uint16_t)compare);
}

void calculate_servo_pulse(float right_speed, float left_speed, float *right_pulse, float *left_pulse) {
    // 右モータのパルス幅計算
    float right_pulse_us = SERVO_NEUTRAL_RIGHT + right_speed;
    if (right_pulse_us > SERVO_MAX_RIGHT) right_pulse_us = SERVO_MAX_RIGHT;
    if (right_pulse_us < SERVO_MIN_RIGHT) right_pulse_us = SERVO_MIN_RIGHT;
    *right_pulse = right_pulse_us;
    
    // 左モータのパルス幅計算
    float left_pulse_us = SERVO_NEUTRAL_LEFT + left_speed;
    if (left_pulse_us > SERVO_MAX_LEFT) left_pulse_us = SERVO_MAX_LEFT;
    if (left_pulse_us < SERVO_MIN_LEFT) left_pulse_us = SERVO_MIN_LEFT;
    *left_pulse = left_pulse_us;
}