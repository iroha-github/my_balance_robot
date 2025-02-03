#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hcsr04.h"
#include "config.h"

// 音速を用いた1usあたりの移動距離(往復分)を考慮
// 通常340m/s = 34000cm/s
// Echoパルスは往復時間なので 1usあたり約0.034cm/2 = 0.017cm
#define SOUND_SPEED_PER_US 0.017f

void init_hcsr04(void) {
    // Trigピン 出力設定
    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0); // 初期Low

    // Echoピン 入力設定
    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
    // 必要に応じて pull_up/down を設定(回路に合わせる)
}

float measure_distance_cm(void) {
    // 1) Trigピンに短パルス(10us以上)を送信
    gpio_put(ULTRA_TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(ULTRA_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRA_TRIG_PIN, 0);

    // 2) EchoピンがHighになるのを待つ
    absolute_time_t start_wait = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 0) {
        // タイムアウト対策(約200ms程度)
        if (absolute_time_diff_us(start_wait, get_absolute_time()) > 200000) {
            return -1.0f; // タイムアウト(立ち上がり来ない) 
        }
    }
    absolute_time_t echo_start = get_absolute_time();

    // 3) EchoピンがLowになるのを待つ
    while (gpio_get(ULTRA_ECHO_PIN) == 1) {
        // タイムアウト対策(約200ms程度)
        if (absolute_time_diff_us(echo_start, get_absolute_time()) > 200000) {
            return -2.0f; // タイムアウト(立ち下がり来ない)
        }
    }
    absolute_time_t echo_end = get_absolute_time();

    // 4) Echoのパルス幅計算 (us)
    int64_t pulse_us = absolute_time_diff_us(echo_start, echo_end);

    // 5) 音速換算で距離を算出 [cm]
    float distance_cm = (float)pulse_us * SOUND_SPEED_PER_US;
    return distance_cm;
}
