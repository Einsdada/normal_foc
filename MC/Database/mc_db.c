/**
 * @file    mc_db.c
 * @brief   电机控制数据库实现：全局共享实例 mc 定义 + 模式名称
 */
#include "mc_db.h"

/* ==================== 全局共享实例（单电机系统） ==================== */
MC_Handle_t mc = {
    .mode   = MODE_STOP,
    .target = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .state  = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    .device = { 0 },            /* 设备句柄由 Control_Init() 填充初始化 */
    .sched  = { 0u, 0u },
    .enc_calib = { ENC_CALIB_PHASE_LOCK, 0u, 0u, 0.0f, 0.0f, 0.0f, 0u, 0u, 0.0f, 0.0f },
    /* pid / enc_calib_done / calib_report 由 Control_Init() 填充初始化 */
};

/* ==================== 模式名称（调试打印） ==================== */
const char *mc_db_mode_name(mode_t mode)
{
    switch (mode)
    {
    case MODE_STOP:                return "STOP";
    case MODE_CURRENT_CALIB:       return "CURRENT_CALIB";
    case MODE_ENCODER_CALIB:       return "ENCODER_CALIB";
    case MODE_FULL_OPEN_LOOP:      return "FULL_OPEN_LOOP";
    case MODE_HALF_OPEN_LOOP:      return "HALF_OPEN_LOOP";
    case MODE_CURRENT_LOOP:        return "CURRENT_LOOP";
    case MODE_SPEED_CURRENT_LOOP:  return "SPEED_CURRENT_LOOP";
    case MODE_POS_SPEED_CURRENT_LOOP: return "POS_SPEED_CURRENT_LOOP";
    default:                       return "UNKNOWN";
    }
}
