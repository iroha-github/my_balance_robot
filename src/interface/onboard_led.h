#ifndef _ONBOARD_LED_H_
#define _ONBOARD_LED_H_

/**
 * @file onboard_led.h
 * @brief オンボードLEDの初期化および制御の関数群
 */

#include <stdint.h>
#include <stdbool.h>  // bool型のために追加

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 0
#endif

/**
 * @brief オンボードLEDの初期化を行います。
 *
 * @return 0 成功、負の値 エラー
 */
int pico_led_init(void);

/**
 * @brief オンボードLEDの状態を設定します。
 *
 * @param status true でLED ON、false でLED OFF
 */
void pico_set_led(bool status);

#ifdef __cplusplus
}
#endif

#endif /* _ONBOARD_LED_H_ */
