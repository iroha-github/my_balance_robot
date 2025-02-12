#ifndef COMMAND_INPUT_H
#define COMMAND_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief シリアル入力により前後左右の指令値を保持する構造体
 *        - pitch_offset: 前後動作用（目標ピッチ角のオフセット、単位：deg）
 *        - turn_offset : 左右動作用（左右サーボ出力の差分、単位：µs）
 */
typedef struct {
    float pitch_offset;   // 前後（前進/後退）用オフセット（deg）
    float turn_offset;    // 左右（旋回）用オフセット（µs）
} CommandInput_t;

/**
 * @brief コマンド入力の初期化（※今回の例では stdio_init_all() を main で行っているため特に処理は不要）
 */
void init_command_input(void);

/**
 * @brief 非ブロッキングにシリアル入力をチェックし、コマンドに応じたオフセット値を更新する
 * @param cmd [in,out] CommandInput_t 構造体のポインタ
 */
void update_command_input(CommandInput_t *cmd);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_INPUT_H
