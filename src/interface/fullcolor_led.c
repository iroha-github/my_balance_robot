#include "fullcolor_led.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "config.h"  // ここで RGBピンの設定を読み込む

// 内部的にPWMチャネルレベルを設定する関数
static void pwm_set_level_for_led(uint slice_num, uint channel, uint16_t level) {
    pwm_set_chan_level(slice_num, channel, level);
}

/**
 * @brief フルカラーLEDの初期化
 */
void fullcolor_led_init(void) {
    // それぞれのピンをPWM出力に設定
    gpio_set_function(FULLCOLOR_LED_R_PIN, GPIO_FUNC_PWM);
    gpio_set_function(FULLCOLOR_LED_G_PIN, GPIO_FUNC_PWM);
    gpio_set_function(FULLCOLOR_LED_B_PIN, GPIO_FUNC_PWM);

    // PWMスライスを取得
    uint slice_r = pwm_gpio_to_slice_num(FULLCOLOR_LED_R_PIN);
    uint slice_g = pwm_gpio_to_slice_num(FULLCOLOR_LED_G_PIN);
    uint slice_b = pwm_gpio_to_slice_num(FULLCOLOR_LED_B_PIN);

    // デフォルトPWM設定を取得
    pwm_config config_r = pwm_get_default_config();
    pwm_config config_g = pwm_get_default_config();
    pwm_config config_b = pwm_get_default_config();

    // 例: クロック分周・ラップ値などの設定
    // ここでは 8bit程度(0~255)で制御したいので wrapを255に設定する例
    pwm_config_set_wrap(&config_r, 255);
    pwm_config_set_wrap(&config_g, 255);
    pwm_config_set_wrap(&config_b, 255);

    // 例として分周を1にすれば、システムクロック125MHzをそのまま使うと非常に高速になるため、
    // 適宜分周を大きくしたり、さらにラップ値を増やしたりして周波数を調整してください。
    // pwm_config_set_clkdiv(&config_r, 256.0f); // 必要に応じて調整
    // pwm_config_set_clkdiv(&config_g, 256.0f);
    // pwm_config_set_clkdiv(&config_b, 256.0f);

    // 初期化
    pwm_init(slice_r, &config_r, true);
    pwm_init(slice_g, &config_g, true);
    pwm_init(slice_b, &config_b, true);

    // すべて 0(消灯) にする
    set_fullcolor_led_rgb(0, 0, 0);
}

/**
 * @brief RGB値を指定してLEDを制御する
 * @param r 0~255
 * @param g 0~255
 * @param b 0~255
 */
void set_fullcolor_led_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // フルカラーLEDがアノード・カソードどちらかによって極性が変わります。
    // 以下は **共通アノード** タイプの場合の例で、出力が小さいほど明るくなる想定：
    //   (アノード=VCC, RGBピンからGNDへ電流を流す)
    // 共通カソードの場合は逆に 0~255 をそのままレベルにすればOKです。
    // 必要に応じて invert(255 - r) などして下さい。

    // 例: 共通カソード(0でOFF, 255で最大点灯)と仮定する
    uint slice_r = pwm_gpio_to_slice_num(FULLCOLOR_LED_R_PIN);
    uint slice_g = pwm_gpio_to_slice_num(FULLCOLOR_LED_G_PIN);
    uint slice_b = pwm_gpio_to_slice_num(FULLCOLOR_LED_B_PIN);

    uint channel_r = pwm_gpio_to_channel(FULLCOLOR_LED_R_PIN);
    uint channel_g = pwm_gpio_to_channel(FULLCOLOR_LED_G_PIN);
    uint channel_b = pwm_gpio_to_channel(FULLCOLOR_LED_B_PIN);

    pwm_set_level_for_led(slice_r, channel_r, r);
    pwm_set_level_for_led(slice_g, channel_g, g);
    pwm_set_level_for_led(slice_b, channel_b, b);
}
