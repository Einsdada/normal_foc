# =====================================================================
# MC 应用层：编译为静态库 mc
# 外层 CMakeLists 通过 add_subdirectory(MC) 引入，直接链接 mc 即可
# 以后在 MC 下新增源文件，只需改这里的列表
# =====================================================================

# 源码列表（MC 下新增 .c 文件时在这里添加）
add_library(mc STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/user.c
    ${CMAKE_CURRENT_SOURCE_DIR}/user_debug.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Led/led_bsp_pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Led/led_bsp_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/MT6701/mt6701_ssi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/MT6701/mt6701_abz.c
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/MT6701/mt6701_i2c_soft.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/pos_sensor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/led.c
)

# 头文件搜索路径
target_include_directories(mc PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/Led
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/MT6701
    ${CMAKE_CURRENT_SOURCE_DIR}/Device
)

# MC 代码依赖 CubeMX 生成的 HAL 头文件与宏定义（USE_HAL_DRIVER 等）
target_link_libraries(mc PUBLIC stm32cubemx)
