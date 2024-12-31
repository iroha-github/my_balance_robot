#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;    // 積分項の蓄積
    float prev_error;  // 前回の偏差(微分項用)
} PID_t;

/**
 * @brief PID制御の初期化
 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief PID計算を行い、制御出力を返す
 * @param setpoint : 目標値 (例: ピッチ目標角 = 0)
 * @param current  : 現在値 (例: ピッチ計測角)
 * @param dt       : 前回呼び出しからの経過時間(秒)
 * @return PID制御量
 */
float PID_Update(PID_t *pid, float setpoint, float current, float dt);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
