/**
  ******************************************************************************
  * @file    flybox.h
  * @brief   FlyBox 配置数据头文件
  ******************************************************************************
  */

#ifndef __FLYBOX_H
#define __FLYBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 类型定义 ========== */

/* 电机索引枚举 */
typedef enum {
    MOTOR_LEFT_TRACK = 0,   /* 左边履带 */
    MOTOR_RIGHT_TRACK,      /* 右边履带 */
    MOTOR_BOTTOM_BELT,      /* 底带 */
    MOTOR_TURNTABLE,        /* 转盘 */
    MOTOR_HOOK,             /* 抓钩 */
    MOTOR_COUNT             /* 电机数量 */
} MotorIndex_t;

/* 飞箱配置结构体 */
typedef struct {
    uint8_t id;                         /* 飞箱ID (1-6) */
    uint8_t motor_sn[MOTOR_COUNT][7];   /* 5个电机的SN码，每个7字节 */
} FlyBoxConfig_t;

/* ========== 宏定义 ========== */

/* 飞箱数量 */
#define FLYBOX_COUNT  6

/* 电机设备号分配 */
#define DEV_LEFT_TRACK    0x01
#define DEV_RIGHT_TRACK   0x02
#define DEV_BOTTOM_BELT   0x03
#define DEV_TURNTABLE     0x04
#define DEV_HOOK          0x05

/* ========== 外部变量 ========== */

/* 所有飞箱配置表 */
extern const FlyBoxConfig_t g_flybox_configs[FLYBOX_COUNT];

/* ========== 飞箱功能接口声明 ========== */

/**
  * @brief  控制左右履带同步移动（前进/后退）
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_TrackMove(float distance_mm);

/**
  * @brief  控制左履带单独移动
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_LeftTrackMove(float distance_mm);

/**
  * @brief  控制右履带单独移动
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_RightTrackMove(float distance_mm);

/**
  * @brief  控制底带位移
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  */
void FlyBox_BeltMove(float distance_mm);

/**
  * @brief  控制转盘旋转
  * @param  angle_deg: 旋转角度（单位：度，正数顺时针，负数逆时针）
  * @retval None
  */
void FlyBox_TurntableRotate(float angle_deg);

/**
  * @brief  控制抓钩抓取
  * @retval None
  */
void FlyBox_HookGrab(void);

/**
  * @brief  控制抓钩放下
  * @retval None
  */
void FlyBox_HookRelease(void);

/**
  * @brief  停止所有电机
  * @retval None
  */
void FlyBox_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLYBOX_H */
