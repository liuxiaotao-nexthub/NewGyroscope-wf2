/**
  ******************************************************************************
  * @file    spi.c
  * @author  应用团队
  * @brief   SPI驱动实现
  *          SPI配置:
  *          - 模式: 模式0 (CPOL=0, CPHA=0)
  *          - MOSI: PB15
  *          - MISO: PB14
  *          - CLK:  PB13
  *          - CS:   PB12 (软件控制)
  ******************************************************************************
  */

/* 包含文件 -------------------------------------------------------------------*/
#include "spi.h"

/* 私有类型定义 ---------------------------------------------------------------*/
/* 私有宏定义 -----------------------------------------------------------------*/
/* 私有变量 -------------------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* 私有函数原型 ---------------------------------------------------------------*/
/* 私有函数 -------------------------------------------------------------------*/

/**
  * @brief  初始化SPI2外设
  * @param  无
  * @retval 无
  */
void SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 使能SPI2和GPIOB时钟 */
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* 配置SPI2 GPIO引脚 */
	/* PB13: SCK, PB14: MISO, PB15: MOSI */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* 配置PB12为软件片选 (输出) */
	GPIO_InitStruct.Pin = SPI_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SPI_CS_GPIO_PORT, &GPIO_InitStruct);

	/* 设置片选为高电平 (空闲状态) */
	SPI_CS_HIGH();

	/* 配置SPI2 */
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;        /* CPOL = 0 */
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;            /* CPHA = 0 */
	hspi2.Init.NSS = SPI_NSS_SOFT;                    /* 软件片选 */
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* APB1=36MHz, SPI=2.25MHz */
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		while (1);
	}
}

/**
  * @brief  通过SPI发送和接收数据
  * @param  pTxData: 发送数据缓冲区指针
  * @param  pRxData: 接收数据缓冲区指针
  * @param  size: 传输数据量
  * @retval SPI_StatusTypeDef 操作状态
  */
SPI_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t size)
{
	HAL_StatusTypeDef status;
	
	status = HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, size, SPI_TIMEOUT_MS);
	
	if (status == HAL_OK)
		return SPI_OK;
	else if (status == HAL_TIMEOUT)
		return SPI_TIMEOUT;
	else
		return SPI_ERROR;
}

/**
  * @brief  通过SPI发送数据
  * @param  pData: 数据缓冲区指针
  * @param  size: 发送数据量
  * @retval SPI_StatusTypeDef 操作状态
  */
SPI_StatusTypeDef SPI_Transmit(uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef status;
	
	status = HAL_SPI_Transmit(&hspi2, pData, size, SPI_TIMEOUT_MS);
	
	if (status == HAL_OK)
		return SPI_OK;
	else if (status == HAL_TIMEOUT)
		return SPI_TIMEOUT;
	else
		return SPI_ERROR;
}

/**
  * @brief  通过SPI接收数据
  * @param  pData: 数据缓冲区指针
  * @param  size: 接收数据量
  * @retval SPI_StatusTypeDef 操作状态
  */
SPI_StatusTypeDef SPI_Receive(uint8_t *pData, uint16_t size)
{
	HAL_StatusTypeDef status;
	
	status = HAL_SPI_Receive(&hspi2, pData, size, SPI_TIMEOUT_MS);
	
	if (status == HAL_OK)
		return SPI_OK;
	else if (status == HAL_TIMEOUT)
		return SPI_TIMEOUT;
	else
		return SPI_ERROR;
}

/************************ 文件结束 ****/
