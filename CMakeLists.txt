# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(super)

# 添加子目录，假设 drivers/can 是当前项目的子目录
add_subdirectory(drivers/can)
add_subdirectory(drivers/gpio)


# 添加motorlib库
add_subdirectory(../modules/lib/motorlib ${CMAKE_BINARY_DIR}/motorlib)

# 添加CommonLibrary库
add_subdirectory(../modules/lib/CommonLibrary ${CMAKE_BINARY_DIR}/CommonLibrary)

# 添加Protocol库
add_subdirectory(../modules/lib/ProtocolV4 ${CMAKE_BINARY_DIR}/ProtocolV4)

# 添加源文件到 app target
target_sources(app PRIVATE
    src/main.c
    src/mc_thread.c
    src/super_thread.c
)