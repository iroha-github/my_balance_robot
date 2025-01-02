// src/blink/blink.h

#ifndef LED_BLINK_H
#define LED_BLINK_H

#include <stdbool.h>
#include "pico/stdlib.h"

// LED が接続されているGPIOピン (Pico W以外など)
#ifndef LED_PIN
    #define LED_PIN 25
#endif

// ステージの数
#define NUM_BLINK_STAGES 3

// 点滅ステージ1つ分の定義
typedef struct {
    uint32_t interval_ms; // 点滅または点灯の間隔（ミリ秒）
    bool     toggle;      // true=点滅, false=常時点灯
} BlinkStage_t;

/**
 * @brief LED点滅モジュールの初期化
 * @return 成功=true, 失敗=false
 */
bool led_blink_init(void);

/**
 * @brief LED点滅モジュールの更新（メインループ内などで定期的に呼ぶ）
 */
void led_blink_update(void);

/**
 * @brief ステージを変更する
 * @param stage 0 ~ NUM_BLINK_STAGES-1
 * @return 成功=true, 失敗=false
 */
bool led_blink_set_stage(uint8_t stage);

#endif // LED_BLINK_H
