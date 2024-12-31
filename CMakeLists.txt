# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(my_balance_robot C CXX ASM)

pico_sdk_init()

# 実行ファイルのソース
add_executable(my_balance_robot
    src/main.c
    src/mpu6050_i2c.c
    src/madgwick_filter.c
    src/pid_controller.c
    src/servo.c
)

# ヘッダファイルのディレクトリ
target_include_directories(my_balance_robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(my_balance_robot
    pico_stdlib
    hardware_i2c
    hardware_pwm
)

pico_add_extra_outputs(my_balance_robot)
