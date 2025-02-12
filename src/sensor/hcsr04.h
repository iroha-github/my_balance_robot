#ifndef _HCSR04_H_
#define _HCSR04_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HC-SR04から距離を計測する関数
 * @return 測定距離[cm] (エラーの場合は負値)
 */
float measure_distance_cm(void);

/**
 * @brief HC-SR04用GPIO初期化を行う
 *        (trig, echoピンの方向設定など)
 */
void hcsr04_init(void);

#ifdef __cplusplus
}
#endif

#endif /* _HCSR04_H_ */
