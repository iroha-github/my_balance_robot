# == DO NOT EDIT THE FOLLOWING LINES ...
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

# C/C++ 言語標準
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# PicoボードをPico Wに指定
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pico SDK をインポート
include(pico_sdk_import.cmake)

project(my_balance_robot C CXX ASM)

# Pico SDK の初期化
pico_sdk_init()

# 実行ファイル
add_executable(my_balance_robot
    src/main.c
    src/blink/blink.c
    src/sensor/mpu6050_i2c.c
    src/filter/madgwick_filter.c
    src/controller/pid_controller.c
    src/servo/servo.c
)

# ヘッダファイルの検索パスを指定
target_include_directories(my_balance_robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/src/blink
    ${CMAKE_CURRENT_LIST_DIR}/src/sensor
    ${CMAKE_CURRENT_LIST_DIR}/src/filter
    ${CMAKE_CURRENT_LIST_DIR}/src/controller
    ${CMAKE_CURRENT_LIST_DIR}/src/servo
)

# リンクするライブラリ
target_link_libraries(my_balance_robot
    pico_stdlib
    hardware_i2c
    hardware_pwm

    # Pico W のオンボードLEDを使うだけなら "pico_cyw43_arch_none" をリンク
    pico_cyw43_arch_none

    # Wi-Fi スタックを使う場合は下記いずれかに切り替える:
    # pico_cyw43_arch_lwip_poll
    # pico_cyw43_arch_lwip_threadsafe_background
)

# ヘッダファイルの検索パスを指定
# (pico_cyw43_arch_none はヘッダファイルを持たないため、
#  この設定は必須ではないが、IDEの補完機能を使うために追加している)
target_include_directories(${PROJECT_NAME} PRIVATE

    ${CMAKE_CURRENT_LIST_DIR}/include

    ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_arch/include

)


# USBシリアル出力を有効化 (UART出力も同時に使いたいなら適宜変更)
pico_enable_stdio_usb(my_balance_robot 1)
pico_enable_stdio_uart(my_balance_robot 0)

# ELF / BIN / HEX などのファイルを出力
pico_add_extra_outputs(my_balance_robot)
