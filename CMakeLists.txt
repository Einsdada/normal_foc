cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME normal_foc)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# ===================== 编译优化配置 =====================
# 说明：CubeMX 默认在 cmake/gcc-arm-none-eabi.cmake 中给 Debug 用 -O0 -g3（无优化+全调试信息）、
#       Release 用 -Os -g0（体积优化+无调试信息）。
#       本文件只生成一次（CubeMX 重新 Generate 不会覆盖），在这里重新 set 同名变量即可覆盖，
#       无需改动 gcc-arm-none-eabi.cmake。
#
# --- 优化级别（把下面 -O2 换成任一档即可）---
#   -O0  关闭优化：严格按源码执行、单步最准，但最慢、代码最大（CubeMX 默认）
#   -Og  调试友好：比 -O0 快很多，调试体验基本保留（开发调试推荐）
#   -O2  标准优化：函数内联/寄存器分配/指令调度，性能·体积·调试平衡（嵌入式推荐）
#   -O3  激进优化：额外内联/循环展开，性能极致但 flash 占用增大（时序敏感用）
#   -Os  体积优化：优先减小 flash/ram 占用（发布、flash 紧张时用）
# --- 调试信息级别 ---
#   -g0  无调试信息（发布用）
#   -g2  标准调试信息
#   -g3  最全调试信息（含宏定义，IDE 可查看宏）

# Debug 构建（当前默认构建类型）
set(CMAKE_C_FLAGS_DEBUG   "-O2 -g3")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g3")

# Release 构建（构建时传 -DCMAKE_BUILD_TYPE=Release 生效）
set(CMAKE_C_FLAGS_RELEASE   "-Os -g0")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add user application layer (MC)
add_subdirectory(MC)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    mc

    # Add user defined libraries
)

# 启用 printf 浮点格式化（%f），供串口打印 rad/rps 等浮点量
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
