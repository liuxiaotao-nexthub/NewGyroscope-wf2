/**
  ******************************************************************************
  * @file    flybox.c
  * @brief   FlyBox 配置数据与功能接口
  ******************************************************************************
  */

#include "flybox.h"
#include "motor.h"
#include "can.h"
#include <stddef.h>
#include <math.h>

/* ========== 电机比例系数定义 ========== */
/* 履带比例系数 */
#define TRACK_RATIO             1.5625f   /* 履带比例系数 (脉冲/mm) */

/* 底带比例系数 */
#define BELT_RATIO              7.69864f  /* 底带比例系数 (脉冲/mm) */

/* 底带允许的最大活动范围（mm） */
#define BELT_MAX_MM             550.0f

/* 转盘比例系数 */
#define TURNTABLE_RATIO         15.222f   /* 转盘比例系数 (脉冲/度) */

/* 抓钩比例系数 */
#define HOOK_RATIO              7.28178f  /* 抓钩比例系数 (脉冲/度) */

/* ========== 飞箱配置数据 ========== */

/* 所有飞箱的电机SN码配置表 */
const FlyBoxConfig_t g_flybox_configs[FLYBOX_COUNT] = {
    /* 飞箱1 */
    {
        .id = 1,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x17, 0x05, 0x04, 0x07, 0x2C, 0xAF, 0x00},  /* 左边履带 */
            [MOTOR_RIGHT_TRACK] = {0x17, 0x05, 0x04, 0x07, 0x5A, 0xAF, 0x00},  /* 右边履带 */
            [MOTOR_BOTTOM_BELT] = {0x16, 0x0A, 0x09, 0x07, 0x3B, 0x12, 0x00},  /* 底带 */
            [MOTOR_TURNTABLE]   = {0x17, 0x06, 0x1E, 0x07, 0xB3, 0xFC, 0x00},  /* 转盘 */
            [MOTOR_HOOK]        = {0x17, 0x06, 0x1D, 0x07, 0x04, 0xFA, 0x00},  /* 抓钩 */
        }
    },
    /* 飞箱2 */
    {
        .id = 2,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x17, 0x0B, 0x1B, 0x07, 0x85, 0x15, 0x00},  /* 左边履带 */
            [MOTOR_RIGHT_TRACK] = {0x17, 0x0B, 0x1C, 0x07, 0xF3, 0x17, 0x00},  /* 右边履带 */
            [MOTOR_BOTTOM_BELT] = {0x18, 0x03, 0x1C, 0x07, 0xDA, 0xFD, 0x00},  /* 底带 */
            [MOTOR_TURNTABLE]   = {0x19, 0x0C, 0x03, 0x07, 0x68, 0x2D, 0x00},  /* 转盘 */
            [MOTOR_HOOK]        = {0x19, 0x0C, 0x03, 0x07, 0x8D, 0x2D, 0x00},  /* 抓钩 */
        }
    },
    /* 飞箱3 */
    {
        .id = 3,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x17, 0x05, 0x04, 0x07, 0x26, 0xAF, 0x00},  /* 左边履带 */
            [MOTOR_RIGHT_TRACK] = {0x17, 0x05, 0x04, 0x07, 0x2D, 0xAF, 0x00},  /* 右边履带 */
            [MOTOR_BOTTOM_BELT] = {0x18, 0x06, 0x19, 0x07, 0x72, 0x6C, 0x00},  /* 底带 */
            [MOTOR_TURNTABLE]   = {0x17, 0x06, 0x1E, 0x07, 0x6D, 0xFA, 0x00},  /* 转盘 */
            [MOTOR_HOOK]        = {0x16, 0x07, 0x01, 0x07, 0xCF, 0xC2, 0x00},  /* 抓钩 */
        }
    },
};

/* ========== 飞箱功能接口实现 ========== */

/* 全局记录底带当前位置（单位：mm），默认 300 mm (30 cm) */
float g_belt_position_mm = 300.0f;


/**
  * @brief  控制左右履带同步移动（前进/后退）
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  * @note   左履带：正数前进，负数后退
  *         右履带：与左履带相反方向（负数前进，正数后退）
  */
void FlyBox_TrackMove(float distance_mm)
{
    /* 计算发送给电机的脉冲值 */
    float pulse = distance_mm * TRACK_RATIO;
    
    /* 左履带：正向发送 */
    Motor_SendDisplacement(DEV_LEFT_TRACK, pulse);
    
    /* 右履带：反向发送（前进时右履带实际是后退方向） */
    Motor_SendDisplacement(DEV_RIGHT_TRACK, -pulse);
}

/**
  * @brief  控制左履带单独移动
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_LeftTrackMove(float distance_mm)
{
    float pulse = distance_mm * TRACK_RATIO;
    Motor_SendDisplacement(DEV_LEFT_TRACK, pulse);
}

/**
  * @brief  控制右履带单独移动
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  * @note   右履带方向与左履带相反
  */
void FlyBox_RightTrackMove(float distance_mm)
{
    /* 右履带方向相反 */
    float pulse = -distance_mm * TRACK_RATIO;
    Motor_SendDisplacement(DEV_RIGHT_TRACK, pulse);
}

/**
  * @brief  控制底带位移
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_BeltMove(float distance_mm)
{
    /* 目标新位置（mm） */
    float target_pos = g_belt_position_mm + distance_mm;

    /* 限制目标位置在 [0, BELT_MAX_MM] 范围内 */
    if (target_pos > BELT_MAX_MM) target_pos = BELT_MAX_MM;
    if (target_pos < 0.0f) target_pos = 0.0f;

    /* 实际需要移动的距离（mm） */
    float actual_move = target_pos - g_belt_position_mm;

    /* 如果实际移动为 0 则直接返回 */
    if (fabsf(actual_move) < 0.001f) return;

    float pulse = actual_move * BELT_RATIO;
    Motor_SendDisplacement(DEV_BOTTOM_BELT, pulse);
    /* 更新全局位置（单位：mm） */
    g_belt_position_mm = target_pos;
}

/**
  * @brief  控制转盘旋转
  * @param  angle_deg: 旋转角度（单位：度，正数顺时针，负数逆时针）
  * @retval None
  */
void FlyBox_TurntableRotate(float angle_deg)
{
    float pulse = angle_deg * TURNTABLE_RATIO;
    Motor_SendDisplacement(DEV_TURNTABLE, pulse);
}

/**
  * @brief  控制抓钩旋转
  * @param  angle_deg: 旋转角度（单位：度，正数顺时针，负数逆时针）
  * @retval None
  */
void FlyBox_HookRotate(float angle_deg)
{
    float pulse = angle_deg * HOOK_RATIO;
    Motor_SendDisplacement(DEV_HOOK, pulse);
}

/**
    * @brief 将底带回到默认初始位置（默认 300 mm）
    * @retval None
    */
void FlyBox_BeltHome(void)
{
        /* 目标默认位置为 300 mm */
        float home_mm = 300.0f;
        /* 计算需要移动的量，并调用 FlyBox_BeltMove 进行安全移动（函数内部已做限位） */
        float delta = home_mm - g_belt_position_mm;
        if (fabsf(delta) < 0.001f) return;
        FlyBox_BeltMove(delta);
}

/* ===== TOF 测距相关接口实现 ===== */

/**
  * @brief  发送 TOF 测距请求（ID 0x408）
  * @param  dev_id: TOF 设备号
  * @retval None
  */
void FlyBox_RequestTOF(uint8_t dev_id)
{
    uint8_t cmd[8] = {0};
    cmd[0] = dev_id;
    cmd[1] = 0x0C;
    /* 其余字节保持为 0 */
    CAN_SendData(0x408, cmd, 8);
}

/**
  * @brief  解析 TOF 返回帧（ID 0x409）
  * @param  data: 指向 CAN 数据区
  * @param  len: 数据长度
  * @param  dev_id: 输出设备号（可为NULL）
  * @param  dist_01mm: 输出距离（0.1mm 单位，可为NULL）
  * @param  type: 输出数据类型（0x00 旧数据，0x01 新数据，可为NULL）
  * @retval 0 成功，-1 失败
  */
int FlyBox_ParseTOF(const uint8_t *data, uint8_t len, uint8_t *dev_id, float *dist_mm, uint8_t *type)
{
    /* 要求至少包含 5 字节：dev + 2字节距离 + 1字节类型 (在当前实现中位于 data[4]) */
    if (!data || len < 5) return -1;

    if (dev_id) *dev_id = data[0];

    /* 距离为两字节小端，单位 0.1mm -> 转换为 mm (float) */
    uint16_t d = (uint16_t)(data[2] | (data[3] << 8));

    /* 如果 data[4] == 0，视为无效数据（不可用） */
    if (data[4] == 0) return -1;

    /* 9999 (0.1mm 单位 -> 999.9mm) 视为无效占位值 */
    if (d == 9999) return -1;

    if (dist_mm) {
        *dist_mm = ((float)d) / 10.0f; /* 转换为 mm */
    }

    if (type) {
        *type = data[4];
    }

    return 0;
}

/**
  * @brief  停止所有电机
  * @retval None
  */
void FlyBox_StopAll(void)
{
    Motor_Stop(DEV_LEFT_TRACK);
    Motor_Stop(DEV_RIGHT_TRACK);
    Motor_Stop(DEV_BOTTOM_BELT);
    Motor_Stop(DEV_TURNTABLE);
    Motor_Stop(DEV_HOOK);
}
