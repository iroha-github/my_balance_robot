#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  HC-SR04の初期化(Trig/Echoピンの方向設定など)
 */
void ultrasonic_init(void);

/**
 * @brief  HC-SR04で距離を計測し、[cm]単位で返す
 * @return 距離(cm). タイムアウト時などは -1.0f を返す
 */
float get_distance_cm(void);

#ifdef __cplusplus
}
#endif

#endif // ULTRASONIC_H
