#include "pid_controller.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float current, float dt) {
    float error = setpoint - current; // 偏差 = 目標 - 実測
    pid->integral += error * dt;      // 積分
    float derivative = (error - pid->prev_error) / dt; // 微分

    // PID出力
    float output = pid->Kp * error
                 + pid->Ki * pid->integral
                 + pid->Kd * derivative;

    pid->prev_error = error;

    return output;
}
