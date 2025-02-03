#ifndef FULLCOLOR_LED_H
#define FULLCOLOR_LED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief フルカラーLEDの初期化関数
 *        それぞれのRGBピンをPWM出力として初期化します。
 */
void fullcolor_led_init(void);

/**
 * @brief RGB値を指定してLEDの色を設定する関数
 * @param r 0~255
 * @param g 0~255
 * @param b 0~255
 *
 *  例: set_fullcolor_led_rgb(255, 0, 0) => 赤色
 *      set_fullcolor_led_rgb(0, 255, 0) => 緑色
 *      set_fullcolor_led_rgb(0, 0, 255) => 青色
 */
void set_fullcolor_led_rgb(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif // FULLCOLOR_LED_H
