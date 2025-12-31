/**
  ******************************************************************************
  * @file    can.h
  * @author  应用团队
  * @brief   CAN驱动头文件
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含文件 -------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 导出变量 -------------------------------------------------------------------*/
extern volatile uint8_t CAN_RxFlag;
extern uint8_t CAN_RxData[8];
extern uint32_t CAN_RxDataLen;
extern uint32_t CAN_RxStdId; /* 最近接收帧的标准ID */

/* 导出函数 -------------------------------------------------------------------*/
void CAN_Init(void);
void CAN_SendData(uint32_t id, uint8_t *pData, uint8_t len);
int CAN_AddFilterForId(uint16_t id);
void CAN_ParseRemainingDisplacement(const uint8_t *data, uint32_t *main_rem, uint32_t *slave_rem);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */

/************************ 文件结束 ****/
