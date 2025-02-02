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
cmake_minimum_required(VERSION 3.13) # 必須／cmakeの最低保証バージョンを明記

set(PROJECT_VERSION 1.0.0) # 任意／プロジェクトのバージョンを定義
set(CMAKE_C_STANDARD 11) # 必須?／C言語のバージョンを定義
set(CMAKE_CXX_STANDARD 17) # 必須?／C++言語のバージョンを定義
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 任意?／コンパイルコマンドを出力するかどうかを定義
# set(CMAKE_CXX_STANDARD_REQUIRED ON) # 任意?／C++言語のバージョンを必須にするかどうかを定義
# set(CMAKE_CXX_EXTENSIONS OFF) # 任意?／C++言語の拡張機能を無効にするかどうかを定義

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(my_balance_robot C CXX ASM) # 必須／プロジェクト名を定義
# "PROJECT_NAME"と"PROJECT_SOURSE_DIR"が自動的に設定される

pico_sdk_init()

# find_package(PkgConfig REQUIRED)
# 重要／ライブラリ名を与えると，関連するCmake変数を一括で設定してくれる超便利機能

# 必須／実行ファイルのソース(第一引数をターゲットと呼ぶ)
add_executable(${PROJECT_NAME}
    src/main.c
    src/sensor/mpu6050_i2c.c
    src/filter/madgwick_filter.c
    src/controller/pid_controller.c
    src/servo/servo.c
    src/interface/onboard_led.c
)

# 必須／ヘッダファイルのディレクトリ
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/sensor
    ${CMAKE_CURRENT_LIST_DIR}/src/filter
    ${CMAKE_CURRENT_LIST_DIR}/src/controller
    ${CMAKE_CURRENT_LIST_DIR}/src/servo
    ${CMAKE_CURRENT_LIST_DIR}/src/interface
)

# 必須／リンクするライブラリを指定
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    hardware_pwm
)


pico_add_extra_outputs(${PROJECT_NAME})

message(STATUS "-------------------------------------------------")
message(STATUS "Project Name: ${PROJECT_NAME}")
message(STATUS "PICO_BOARD: ${PICO_BOARD}")
message(STAUTS "Project Version: ${PROJECT_VERSION}")
message(STATUS "-------------------------------------------------")