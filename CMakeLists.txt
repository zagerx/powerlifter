# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(MODULES_BASE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../modules/lib" ABSOLUTE)

# 添加Zephyr外部模块
list(APPEND ZEPHYR_EXTRA_MODULES
    "${MODULES_BASE_PATH}/motorlib"
    "${MODULES_BASE_PATH}/CommonLibrary"
    "${MODULES_BASE_PATH}/ProtocolV4"    
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(super)

# 添加子目录，假设 drivers/can 是当前项目的子目录
add_subdirectory(drivers/can)
add_subdirectory(drivers/gpio)

# 添加源文件到 app target
target_sources(app PRIVATE
    src/main.c
    src/mc_thread.c
    src/super_thread.c
)