#ifndef _MADGWICK_FILTER_H_
#define _MADGWICK_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/**
 * @brief   Madgwickフィルタで用いる状態をまとめた構造体
 */
typedef struct {
    float beta; // フィルタゲイン
    float q0;
    float q1;
    float q2;
    float q3;
} MadgwickFilter_t;

void MadgwickInit(MadgwickFilter_t *filter, float beta);
void MadgwickAHRSupdateIMU(MadgwickFilter_t *mf, 
                           float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt);
void MadgwickGetEulerDeg(const MadgwickFilter_t *mf, float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* _MADGWICK_FILTER_H_ */
