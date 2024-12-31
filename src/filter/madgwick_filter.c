#include "madgwick_filter.h"
#include <stdio.h>

void MadgwickInit(MadgwickFilter_t *filter, float beta) {
    filter->beta = beta;
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
}

// Madgwick 6DoF update
void MadgwickAHRSupdateIMU(MadgwickFilter_t *mf, 
                           float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float dt) {
    float q0 = mf->q0, q1 = mf->q1, q2 = mf->q2, q3 = mf->q3;

    // 加速度が 0ベクトルに近い場合は更新しない
    float norm = ax*ax + ay*ay + az*az;
    if (norm < 1e-8f) {
        // ジャイロからの回転だけを適用(ジャイロバイアス補正等は無しの簡易版)
        float halfDt = 0.5f * dt;
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
        q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
        q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
        q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfDt;
        // 正規化
        norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
        mf->q0 = q0; mf->q1 = q1; mf->q2 = q2; mf->q3 = q3;
        return;
    }

    // 加速度ベクトルを正規化
    norm = sqrtf(norm);
    ax /= norm; ay /= norm; az /= norm;

    // 補正パート (gradient descent algorithm)
    // 参考: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3); // 正規化
    norm = (norm < 1e-8f) ? 1.0f : (1.0f / norm);
    s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

    // ジャイロパート(積分)
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - mf->beta * s0;
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - mf->beta * s1;
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - mf->beta * s2;
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - mf->beta * s3;

    // 更新
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // 正規化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    norm = (norm < 1e-8f) ? 1.0f : (1.0f / norm);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    mf->q0 = q0;
    mf->q1 = q1;
    mf->q2 = q2;
    mf->q3 = q3;
}

// クォータニオン -> オイラー角 (deg)
void MadgwickGetEulerDeg(const MadgwickFilter_t *mf, float *roll, float *pitch, float *yaw) {
    // ZYX オイラー角変換の場合
    float q0 = mf->q0, q1 = mf->q1, q2 = mf->q2, q3 = mf->q3;
    // roll (x軸)
    *roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / M_PI);
    // pitch (y軸)
    float sinp = 2.0f*(q0*q2 - q3*q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI/2.0f, sinp) * (180.0f / M_PI);
    } else {
        *pitch = asinf(sinp) * (180.0f / M_PI);
    }
    // yaw (z軸)
    *yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / M_PI);
}
