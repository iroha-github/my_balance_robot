#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// uint の定義
typedef unsigned int uint;

void init_pwm_for_servo(uint pin, float neutral_us);

void set_servo_pulse(uint pin, float pulse_us);

void calculate_servo_pulse(float pid_output, float *right_pulse, float *left_pulse);

#endif