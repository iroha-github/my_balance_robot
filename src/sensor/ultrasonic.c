#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "config.h"

void ultrasonic_init(void) {
    // Trigピン(出力)
    gpio_init(ULTRA_TRIG_PIN);
    gpio_set_dir(ULTRA_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRA_TRIG_PIN, 0);

    // Echoピン(入力)
    gpio_init(ULTRA_ECHO_PIN);
    gpio_set_dir(ULTRA_ECHO_PIN, GPIO_IN);
}

float get_distance_cm(void) {
    // Trigを10us以上Highに
    gpio_put(ULTRA_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRA_TRIG_PIN, 0);

    // EchoがHighになるまで待機
    absolute_time_t start_time = get_absolute_time();
    while (gpio_get(ULTRA_ECHO_PIN) == 0) {
        if (to_us_since_boot(get_absolute_time()) - to_us_since_boot(start_time) > 30000) {
            return -1.0f; // タイムアウト
        }
    }
    absolute_time_t echo_start = get_absolute_time();

    // EchoがLowになるまで待機
    while (gpio_get(ULTRA_ECHO_PIN) == 1) {
        if (to_us_since_boot(get_absolute_time()) - to_us_since_boot(echo_start) > 30000) {
            return -1.0f; // タイムアウト
        }
    }
    absolute_time_t echo_end = get_absolute_time();

    // パルス幅[us]
    uint64_t pulse_len = to_us_since_boot(echo_end) - to_us_since_boot(echo_start);

    // 往復にかかる時間[us] -> 距離[cm]
    //  音速約340m/s => 往復1cmに約58us
    float distance_cm = (float)pulse_len / 58.0f;
    return distance_cm;
}
