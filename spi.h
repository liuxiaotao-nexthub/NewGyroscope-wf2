/**
  ******************************************************************************
  * @file    spi.h
  * @author  应用团队
  * @brief   SPI驱动头文件
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* 包含文件 -------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 导出类型 -------------------------------------------------------------------*/
typedef enum
{
	SPI_OK = 0,       /* 操作成功 */
	SPI_ERROR = 1,    /* 操作错误 */
	SPI_TIMEOUT = 2   /* 操作超时 */
} SPI_StatusTypeDef;

/* 导出常量 -------------------------------------------------------------------*/
/* SPI片选引脚定义 */
#define SPI_CS_PIN                  GPIO_PIN_12
#define SPI_CS_GPIO_PORT            GPIOB

/* SPI超时时间 (单位:毫秒) */
#define SPI_TIMEOUT_MS              100

/* 导出宏 ---------------------------------------------------------------------*/
#define SPI_CS_LOW()                HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_RESET)
#define SPI_CS_HIGH()               HAL_GPIO_WritePin(SPI_CS_GPIO_PORT, SPI_CS_PIN, GPIO_PIN_SET)

/* 导出函数 -------------------------------------------------------------------*/
void SPI_Init(void);
SPI_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size);
SPI_StatusTypeDef SPI_Transmit(uint8_t *pData, uint16_t size);
SPI_StatusTypeDef SPI_Receive(uint8_t *pData, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */

/************************ 文件结束 ****/
