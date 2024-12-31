// src/servo/servo.c
#include <stdio.h>
#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "config.h" // SERVO_NEUTRAL_US, SERVO_RANGE_US, SERVO_PERIOD_US など

// static void set_servo_pulse(uint pin, float pulse_us);

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
    set_servo_pulse(pin, SERVO_NEUTRAL_US);
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
