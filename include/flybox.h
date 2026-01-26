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

/* 全局记录底带当前位置（单位：cm），默认 30 cm */
extern float g_belt_position_mm;

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
  * @brief  控制底带位移（管理全局位置，用于测试任务）
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  * @note   会更新全局位置 g_belt_position_mm
  */
void FlyBox_BeltMove(float distance_mm);

/**
  * @brief  控制底带原始移动（不管理全局位置，用于回零任务）
  * @param  distance_mm: 位移距离（单位：mm，正数前进，负数后退）
  * @retval None
  * @note   直接移动，不更新 g_belt_position_mm，不做范围限制
  */
void FlyBox_BeltMoveRaw(float distance_mm);

/**
  * @brief  控制转盘旋转
  * @param  angle_deg: 旋转角度（单位：度，正数顺时针，负数逆时针）
  * @retval None
  */
void FlyBox_TurntableRotate(float angle_deg);

/**
  * @brief  控制抓钩旋转
  * @param  angle_deg: 旋转角度（单位：度，正数顺时针，负数逆时针）
  * @retval None
  */
void FlyBox_HookRotate(float angle_deg);

/* 将底带回到默认位置（单位：mm，默认 300 mm） */
void FlyBox_BeltHome(void);

/**
  * @brief  停止所有电机
  * @retval None
  */
void FlyBox_StopAll(void);

/**
  * @brief  发送 TOF 测距模块查询命令（ID 0x408）
  * @param  dev_id: TOF 设备号（例如 0x19 或 0x1A）
  * @retval None
  * @note   发送数据: [dev_id, 0x0C,0,0,0,0,0,0]
  */
void FlyBox_RequestTOF(uint8_t dev_id);

/**
  * @brief  解析 TOF 模块返回的 CAN 帧（ID 0x409）
  * @param  data: 指向 CAN 数据区指针
  * @param  len: 数据长度（应>=4）
  * @param  dev_id: 输出设备号（可为NULL）
  * @param  dist_01mm: 输出距离，单位 0.1mm（可为NULL）
  * @param  type: 输出数据类型（0x00 旧数据，0x01 新数据）（可为NULL）
  * @retval int: 0 表示解析成功，-1 表示数据无效
  */
int FlyBox_ParseTOF(const uint8_t *data, uint8_t len, uint8_t *dev_id, float *dist_mm, uint8_t *type);

#ifdef __cplusplus
}
#endif

#endif /* __FLYBOX_H */
