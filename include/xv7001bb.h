/**
  ******************************************************************************
  * @file    xv7001bb.h
  * @author  应用团队
  * @brief   XV7001BB陀螺仪驱动头文件
  ******************************************************************************
  */

#ifndef __XV7001BB_H
#define __XV7001BB_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含文件 -------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "spi.h"

/* 低功耗命令寄存器 */
#define CMD_SLEEP_IN   0x05  /* 进入睡眠模式 */
#define CMD_SLEEP_OUT  0x06  /* 退出睡眠/待机模式 */
#define CMD_STANDBY    0x07  /* 进入待机模式 */

void XV7001BB_Init(void);
void XV7001BB_WriteReg(uint8_t reg, uint8_t data);
uint8_t XV7001BB_ReadReg(uint8_t reg);

/* 低功耗模式控制 */
void enter_standby_mode(void);
void enter_sleep_mode(void);
void wake_up_gyro(void);

/* 传感器读取接口：默认温度12bit，角速度16bit */
float XV7001BB_ReadTemperature(void);
float XV7001BB_ReadAngularRate(void);

#ifdef __cplusplus
}
#endif

#endif /* __XV7001BB_H */

/************************ 文件结束 ****/
