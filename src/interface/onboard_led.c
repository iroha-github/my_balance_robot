#include "onboard_led.h"

#include "pico/stdlib.h"   // gpio_init, gpio_set_dir, gpio_put などの関数宣言用

#ifdef PICO_DEFAULT_LED_PIN
#include "hardware/gpio.h"
#endif

//cyw43_archのwarningを抑制
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#include <stdio.h>

int onboard_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // LED に GPIO を使用するデバイス（Pico など）は PICO_DEFAULT_LED_PIN を定義します
    // そのため、通常の GPIO 機能を使用して LED をオンおよびオフにすることができます
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Pico W デバイスの場合、ドライバなどを初期化する必要があります
    return cyw43_arch_init(); // cyw43_arch_init()もPICO_OKを返す
#endif
}

void onboard_led_set(bool status) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, status);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // WIFI "ドライバ" に GPIO のオンまたはオフを設定するように依頼します
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, status);
#endif
}
