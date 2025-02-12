cmake_minimum_required(VERSION 3.13)

set(PROJECT_VERSION 1.0.0)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(my_balance_robot C CXX ASM)

pico_sdk_init()

# 実行ファイル
add_executable(${PROJECT_NAME}
    src/main.c
    src/sensor/mpu6050_i2c.c
    src/sensor/hcsr04.c          # ★追加
    src/filter/madgwick_filter.c
    src/controller/pid_controller.c
    src/servo/servo.c
    src/interface/onboard_led.c
    src/interface/rotary_switch.c # ★追加
    src/interface/fullcolor_led.c # ★追加
    src/controller/command_input.c   # ★追加：シリアル入力処理
)

# ヘッダファイルのディレクトリ
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/sensor
    ${CMAKE_CURRENT_LIST_DIR}/src/filter
    ${CMAKE_CURRENT_LIST_DIR}/src/controller
    ${CMAKE_CURRENT_LIST_DIR}/src/servo
    ${CMAKE_CURRENT_LIST_DIR}/src/interface
)

# リンクするライブラリ
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    hardware_pwm
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${PROJECT_NAME} pico_cyw43_arch_none)
endif()

pico_add_extra_outputs(${PROJECT_NAME})
