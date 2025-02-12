#ifndef _ROTARY_SWITCH_H_
#define _ROTARY_SWITCH_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief ロータリスイッチのGPIO初期化
 *        (gpio18,19,20,21の入力設定・PullDownなど)
 */
void rotary_switch_init(void);

/**
 * @brief ロータリスイッチ(4ピン)を読んで現在のモードを返す
 * @return
 *   - 0: GPIO18がHIGH (距離センサ使用)
 *   - 1: GPIO19がHIGH
 *   - 2: GPIO20がHIGH
 *   - 3: GPIO21がHIGH
 *   - -1: (複数/どれもHIGHでない/エラー) 
 */
int read_rotary_switch_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* _ROTARY_SWITCH_H_ */
