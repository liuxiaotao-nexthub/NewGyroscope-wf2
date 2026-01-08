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

#ifdef __cplusplus
}
#endif

#endif /* __FLYBOX_H */
