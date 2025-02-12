#include "command_input.h"
#include "pico/stdlib.h"
#include <stdio.h>

void init_command_input(void) {
    // すでに main() 内で stdio_init_all() を呼んでいるので、ここでは何もする必要はありません。
}

void update_command_input(CommandInput_t *cmd) {
    // 非ブロッキングにシリアル入力をチェック（timeout 0）
    int ch = getchar_timeout_us(0);
    if (ch != PICO_ERROR_TIMEOUT) {
        // 入力された文字に応じてオフセット値を調整
        switch(ch) {
            case 'w': // 前進：前傾（目標ピッチを正方向に増加）
                cmd->pitch_offset += 0.5f;  // ※適宜調整
                printf("Command: w (Forward) -> pitch_offset = %.2f\n", cmd->pitch_offset);
                break;
            case 's': // 後退：後傾（目標ピッチを負方向に減少）
                cmd->pitch_offset -= 0.5f;
                printf("Command: s (Backward) -> pitch_offset = %.2f\n", cmd->pitch_offset);
                break;
            case 'a': // 左折：右モーターを加速、左モーターを減速（turn_offset 正方向）
                cmd->turn_offset += 5.0f;   // 単位はµs（適宜調整）
                printf("Command: a (Left Turn) -> turn_offset = %.1f\n", cmd->turn_offset);
                break;
            case 'd': // 右折：左モーターを加速、右モーターを減速（turn_offset 負方向）
                cmd->turn_offset -= 5.0f;
                printf("Command: d (Right Turn) -> turn_offset = %.1f\n", cmd->turn_offset);
                break;
            case 'x': // オフセットリセット
                cmd->pitch_offset = 0.0f;
                cmd->turn_offset = 0.0f;
                printf("Command: x (Reset Offsets)\n");
                break;
            default:
                // その他の入力は無視
                break;
        }
    }
}
