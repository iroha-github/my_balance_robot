// src/blink/blink.c

#include <stdio.h>
#include "blink.h"

// Pico W 用のヘッダ：オンボードLEDを使うには cyw43_arch へのリンクが必要
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// 点滅ステージの定義（必要に応じて変更）
static BlinkStage_t stages[NUM_BLINK_STAGES] = {
    { 500, true  }, // ステージ0: 500ms間隔で点滅
    { 200, true  }, // ステージ1: 200ms間隔で点滅
    {1000, true }  // ステージ2: 1000ms間隔で点灯（toggle=false → 点滅しない）
};

// 現在のステージ
static uint8_t current_stage = 0;

// 前回の切り替え時刻
static absolute_time_t last_toggle_time;

// LEDの現在の状態
static bool led_state = false;

/**
 * @brief LEDをオンまたはオフにする内部関数
 */
static void set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Pico(W 以外) のデフォルトLEDピン (通常25番ピン)
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Pico W のオンボードLED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#else
    // それ以外のカスタムピン
    gpio_put(LED_PIN, led_on);
#endif
}

/**
 * @brief LED点滅モジュールの初期化
 */
bool led_blink_init(void) {
    int rc = PICO_OK;
    
#if defined(PICO_DEFAULT_LED_PIN)
    // Pico (W以外) の場合など
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    set_led(false);

#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Pico W の場合 (オンボードLED操作)
    rc = cyw43_arch_init();  // pico_cyw43_arch_none 等をリンクしている場合
    if (rc != 0) {
        printf("cyw43_arch_init failed: %d\n", rc);
        return false;
    }
    // Wi-Fi を使わないなら STA モードを有効にする必要はありません。必要に応じてコメントアウトしてください。
    // cyw43_arch_enable_sta_mode();

    set_led(false);

#else
    // カスタムピンの場合
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    set_led(false);
#endif

    last_toggle_time = get_absolute_time();
    led_state = false;

    return true;
}

/**
 * @brief LED点滅を制御する。メインループなどで定期的に呼び出す。
 */
void led_blink_update(void) {
    absolute_time_t now = get_absolute_time();
    uint32_t elapsed_ms = (uint32_t)(absolute_time_diff_us(last_toggle_time, now) / 1000);

    uint32_t interval = stages[current_stage].interval_ms;

    if (elapsed_ms >= interval) {
        last_toggle_time = now;

        if (stages[current_stage].toggle) {
            // 点滅パターン
            led_state = !led_state;
            set_led(led_state);
        } else {
            // 点灯パターン (常に ON)
            set_led(true);
        }
    }
}

/**
 * @brief LEDの点滅ステージを変更
 * @param stage 変更先ステージ (0 ~ NUM_BLINK_STAGES-1)
 */
bool led_blink_set_stage(uint8_t stage) {
    if (stage >= NUM_BLINK_STAGES) {
        return false; // 無効なステージ番号
    }
    current_stage = stage;
    // 即時反映するためにカウンタなどをリセット
    last_toggle_time = get_absolute_time();
    if (!stages[current_stage].toggle) {
        // toggle=false のときは点滅しないので、常に点灯状態にする
        set_led(true);
        led_state = true;
    } else {
        // トグル式の場合は現在のled_stateを保持したまま
    }
    printf("LEDステージを %d に変更しました。\n", current_stage);
    return true;
}
