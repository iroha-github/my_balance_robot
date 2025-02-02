#include "rotary_switch.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "config.h" // ROTARY_SW_PIN1, ROTARY_SW_PIN2, ROTARY_SW_PIN3, ROTARY_SW_PIN4

void init_rotary_switch(void) {
    // GPIO18,19,20,21 入力設定 (例: PullDown)
    gpio_init(ROTARY_SW_PIN1);
    gpio_set_dir(ROTARY_SW_PIN1, GPIO_IN);
    gpio_pull_down(ROTARY_SW_PIN1);

    gpio_init(ROTARY_SW_PIN2);
    gpio_set_dir(ROTARY_SW_PIN2, GPIO_IN);
    gpio_pull_down(ROTARY_SW_PIN2);

    gpio_init(ROTARY_SW_PIN3);
    gpio_set_dir(ROTARY_SW_PIN3, GPIO_IN);
    gpio_pull_down(ROTARY_SW_PIN3);

    gpio_init(ROTARY_SW_PIN4);
    gpio_set_dir(ROTARY_SW_PIN4, GPIO_IN);
    gpio_pull_down(ROTARY_SW_PIN4);
}

int read_rotary_switch_mode(void) {
    bool s18 = gpio_get(ROTARY_SW_PIN1);
    bool s19 = gpio_get(ROTARY_SW_PIN2);
    bool s20 = gpio_get(ROTARY_SW_PIN3);
    bool s21 = gpio_get(ROTARY_SW_PIN4);

    // シンプルに1つだけHIGH
    if (s18 && !s19 && !s20 && !s21) return 0; // GPIO18
    if (!s18 && s19 && !s20 && !s21) return 1; // GPIO19
    if (!s18 && !s19 && s20 && !s21) return 2; // GPIO20
    if (!s18 && !s19 && !s20 && s21) return 3; // GPIO21

    // それ以外はエラー
    return -1;
}
