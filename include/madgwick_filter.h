#ifndef _MADGWICK_FILTER_H_
#define _MADGWICK_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/**
 * @brief   Madgwickフィルタで用いる状態をまとめた構造体
 *          ここで姿勢はクォータニオン(q0, q1, q2, q3)で保持
 */
typedef struct {
    float beta; // フィルタゲイン (2 * proportional gain)
    float q0;
    float q1;
    float q2;
    float q3;
} MadgwickFilter_t;

/**
 * @brief   Madgwickフィルタ構造体を初期化する(クォータニオンを単位四元数に、betaを設定)
 * @param   filter     [out] 構造体
 * @param   beta       [in]  フィルタゲイン (推奨 0.01~0.1程度・チューニングが必要)
 */
void MadgwickInit(MadgwickFilter_t *filter, float beta);

/**
 * @brief   加速度・ジャイロからクォータニオンを更新する(6軸)
 * @param   mf       [in,out] MadgwickFilter_t構造体
 * @param   gx, gy, gz  ジャイロ角速度 [rad/s]
 * @param   ax, ay, az  加速度 [g] (単位ベクトル推奨)
 * @param   dt          前回呼び出しからの経過時間(秒)
 */
void MadgwickAHRSupdateIMU(MadgwickFilter_t *mf, 
                           float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);

/**
 * @brief   フィルタ構造体内のクォータニオンからオイラー角(ピッチ, ロール, ヨー)を[deg]単位で返す
 *          RPY順(Euler angles) : 
 *            - roll  = 回転軸 X
 *            - pitch = 回転軸 Y
 *            - yaw   = 回転軸 Z
 */
void MadgwickGetEulerDeg(const MadgwickFilter_t *mf, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* _MADGWICK_FILTER_H_ */
