/**
  ******************************************************************************
  * @file    flybox.c
  * @brief   FlyBox 配置数据与功能接口
  ******************************************************************************
  */

#include "flybox.h"
#include "motor.h"

/* ========== 电机比例系数定义 ========== */
/* 履带：15625 对应 1000mm，比例 = 15.625 */
#define TRACK_RATIO             15.625f   /* 履带比例系数 (脉冲/mm) */

/* 底带：76986.4 对应 1000mm，比例 = 76.9864 */
#define BELT_RATIO              76.9864f  /* 底带比例系数 (脉冲/mm) */

/* 转盘：13700 对应 90°，比例 = 152.22 */
#define TURNTABLE_RATIO         152.22f   /* 转盘比例系数 (脉冲/度) */

/* 抓钩：6553.6 为抓取位置 */
#define HOOK_GRAB_POSITION      6553.6f   /* 抓钩抓取位置 */

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
            [MOTOR_BOTTOM_BELT] = {0x17, 0x0B, 0x02, 0x07, 0xE7, 0xCE, 0x00},  /* 底带 */
            [MOTOR_TURNTABLE]   = {0x18, 0x03, 0x1C, 0x07, 0xDA, 0xFD, 0x00},  /* 转盘 */
            [MOTOR_HOOK]        = {0x19, 0x0C, 0x03, 0x07, 0x8F, 0x2D, 0x00},  /* 抓钩 */
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
    /* 飞箱4 - 待配置 */
    {
        .id = 4,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_RIGHT_TRACK] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_BOTTOM_BELT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_TURNTABLE]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_HOOK]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        }
    },
    /* 飞箱5 - 待配置 */
    {
        .id = 5,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_RIGHT_TRACK] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_BOTTOM_BELT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_TURNTABLE]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_HOOK]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        }
    },
    /* 飞箱6 - 待配置 */
    {
        .id = 6,
        .motor_sn = {
            [MOTOR_LEFT_TRACK]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_RIGHT_TRACK] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_BOTTOM_BELT] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_TURNTABLE]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
            [MOTOR_HOOK]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        }
    },
};

/* ========== 飞箱功能接口实现 ========== */

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
    float pulse = distance_mm * BELT_RATIO;
    Motor_SendDisplacement(DEV_BOTTOM_BELT, pulse);
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
  * @brief  控制抓钩抓取
  * @retval None
  */
void FlyBox_HookGrab(void)
{
    Motor_SendDisplacement(DEV_HOOK, -HOOK_GRAB_POSITION);
}

/**
  * @brief  控制抓钩放下
  * @retval None
  */
void FlyBox_HookRelease(void)
{
    Motor_SendDisplacement(DEV_HOOK, HOOK_GRAB_POSITION);
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
