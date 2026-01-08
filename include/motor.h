/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机控制模块头文件
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 宏定义 ========== */

/* 电机设备号 */
#define MOTOR_DEV_1  0x01
#define MOTOR_DEV_2  0x02

/* 电机使能/失能 */
#define MOTOR_ENABLE   0x5A
#define MOTOR_DISABLE  0x4A

/* ========== 函数声明 ========== */

/**
  * @brief  请求电机SN码（ID 0x60D）
  * @retval None
  */
void Motor_RequestSN(void);

/**
  * @brief  绑定电机SN（ID 0x313）
  * @param  sn: 7字节SN号
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_BindSN(const uint8_t *sn, uint8_t dev_id);

/**
  * @brief  初始化电机参数（ID 0x316）
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_InitParams(uint8_t dev_id);

/**
  * @brief  电机使能/失能（ID 0x413）
  * @param  enable: MOTOR_ENABLE 或 MOTOR_DISABLE
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_Enable(uint8_t enable, uint8_t dev_id);

/**
  * @brief  解析电机位移数据（ID 0x409 响应）
  * @param  data: 8字节CAN数据
  * @param  dev_id: 输出设备号（可为NULL）
  * @retval 位移值（单位：mm）
  */
float Motor_ParsePosition(const uint8_t *data, uint8_t *dev_id);

/**
  * @brief  请求电机返回位移值（ID 0x408，电机通过 0x409 响应）
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_RequestPosition(uint8_t dev_id);

/**
  * @brief  发送位移命令（ID 0x416）
  * @param  dev_id: 设备号
  * @param  displacement: 位移值（单位：mm，正负表示方向）
  * @retval None
  */
void Motor_SendDisplacement(uint8_t dev_id, float displacement);

/**
  * @brief  停止电机
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_Stop(uint8_t dev_id);

/**
  * @brief  设置轮径（ID 0x408）
  * @param  dev_id: 设备号
  * @param  diameter: 轮径（单位：0.01mm）
  * @retval None
  */
void Motor_SetWheelDiameter(uint8_t dev_id, uint16_t diameter);

/**
  * @brief  设置加速度（ID 0x414）
  * @param  dev_id: 设备号
  * @param  acceleration: 加速度（单位：0.1mm/s²）
  * @retval None
  */
void Motor_SetAcceleration(uint8_t dev_id, uint32_t acceleration);

/**
  * @brief  设置速度（ID 0x415）
  * @param  dev_id: 设备号
  * @param  velocity: 速度（单位：0.1mm/s）
  * @retval None
  */
void Motor_SetVelocity(uint8_t dev_id, uint32_t velocity);

/**
  * @brief  设置从位移（ID 0x419）
  * @param  dev_id: 设备号
  * @param  displacement: 从位移（单位：mm）
  * @retval None
  */
void Motor_SetSlaveDisplacement(uint8_t dev_id, float displacement);

/**
  * @brief  设置从速度（ID 0x418）
  * @param  dev_id: 设备号
  * @param  velocity: 从速度（单位：0.1mm/s）
  * @retval None
  */
void Motor_SetSlaveVelocity(uint8_t dev_id, uint32_t velocity);

/**
  * @brief  设置从加速度（ID 0x417）
  * @param  dev_id: 设备号
  * @param  acceleration: 从加速度（单位：0.1mm/s²）
  * @retval None
  */
void Motor_SetSlaveAcceleration(uint8_t dev_id, uint32_t acceleration);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
