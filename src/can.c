/**
  ******************************************************************************
  * @file    can.c
  * @author  应用团队
  * @brief   CAN驱动实现
  *          CAN配置:
  *          - CAN_TX: PA12
  *          - CAN_RX: PA11
  *          - 波特率: 1 Mbps (APB1时钟36MHz)
  *          - 中断接收使能
  ******************************************************************************
  */

/* 包含文件 -------------------------------------------------------------------*/
#include "can.h"

/* 私有类型定义 ---------------------------------------------------------------*/
/* 私有宏定义 -----------------------------------------------------------------*/
/* 私有变量 -------------------------------------------------------------------*/
CAN_HandleTypeDef hcan;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

/* 导出变量 -------------------------------------------------------------------*/
volatile uint8_t CAN_RxFlag = 0;
uint8_t CAN_RxData[8] = {0};
uint32_t CAN_RxDataLen = 0;
uint32_t CAN_RxStdId = 0;

/* 私有函数原型 ---------------------------------------------------------------*/
static void CAN_FilterConfig(void);

/* 动态添加过滤器以接收单个标准ID（在运行时调用） */
int CAN_AddFilterForId(uint16_t id)
{
	CAN_FilterTypeDef sFilterConfig;

	/* 使用下一个可用过滤器槽（这里使用 FilterBank = 1）
	   注意：如果 BSP/其他代码使用更多过滤器，需要管理 bank 号。 */
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	/* 将单个ID放在高16位/低16位条目之一（16位标度，需要左移5位） */
	sFilterConfig.FilterIdHigh = (id << 5) & 0xFFFF;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = (id << 5) & 0xFFFF;
	sFilterConfig.FilterMaskIdLow = 0;

	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		return -1;
	}
	return 0;
}

/* 私有函数 -------------------------------------------------------------------*/

/**
  * @brief  初始化CAN外设
  * @param  无
  * @retval 无
  */
void CAN_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* 使能CAN1和GPIOA时钟 */
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* 配置CAN GPIO引脚 */
	/* PA11: CAN_RX, PA12: CAN_TX */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 配置CAN */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2;                      /* APB1=36MHz, CAN时钟=36/2=18MHz */
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_15TQ;            /* 时间段1: 15TQ */
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;             /* 时间段2: 2TQ */
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;

	/* 波特率计算: 36MHz / (Prescaler * (1 + BS1 + BS2))
	 *           = 36MHz / (2 * (1 + 15 + 2))
	 *           = 36MHz / (2 * 18)
	 *           = 36MHz / 36
	 *           = 1MHz
	 */

	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		while (1);
	}

	/* 配置CAN过滤器 */
	CAN_FilterConfig();

	/* 启动CAN */
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		while (1);
	}

	/* 激活CAN接收通知 */
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		while (1);
	}

	/* 配置CAN中断 - 优先级设置为6，确保低于FreeRTOS最大中断优先级(5) */
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/**
  * @brief  配置CAN过滤器
  * @param  无
  * @retval 无
  */
static void CAN_FilterConfig(void)
{
	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;     /* 列表模式 */
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;    /* 16位标度 */
	
	/* 配置接收的标准ID（需要左移5位） */
	sFilterConfig.FilterIdHigh = (0x312 << 5);            /* ID 0x312 */
	sFilterConfig.FilterIdLow = (0x409 << 5);             /* ID 0x409 */
	sFilterConfig.FilterMaskIdHigh = (0x312 << 5);        /* ID 0x312 (重复) */
	sFilterConfig.FilterMaskIdLow = (0x409 << 5);         /* ID 0x409 (重复) */
	
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		while (1);
	}
}

/**
  * @brief  通过CAN发送数据
  * @param  id: CAN消息ID
  * @param  pData: 数据缓冲区指针
  * @param  len: 数据长度 (0-8字节)
  * @retval 无
  */
void CAN_SendData(uint32_t id, uint8_t *pData, uint8_t len)
{
	uint32_t TxMailbox;
	HAL_StatusTypeDef status;
	uint32_t timeout = 100;  /* 最多等待100次 × 1ms = 100ms */

	/* 准备发送头 */
	TxHeader.StdId = id;
	TxHeader.ExtId = 0x00;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = len;
	TxHeader.TransmitGlobalTime = DISABLE;

	/* 尝试发送CAN消息，如果邮箱满则等待重试 */
	do {
		status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, pData, &TxMailbox);
		if (status == HAL_BUSY) {
			/* 发送邮箱满，延时1ms后重试 */
			HAL_Delay(1);
			timeout--;
		}
	} while (status == HAL_BUSY && timeout > 0);
	
	if (status != HAL_OK) {
		/* 发送失败 */
	}
}

/**
  * @brief  CAN接收FIFO0消息挂起回调函数
  * @param  hcan: CAN句柄指针
  * @retval 无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	/* 获取接收消息 */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxData) == HAL_OK)
	{
		CAN_RxDataLen = RxHeader.DLC;
		CAN_RxStdId = RxHeader.StdId;
		CAN_RxFlag = 1;
	}
}

/**
  * @brief  CAN RX0中断处理函数
  * @param  无
  * @retval 无
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan);
}

/**
  * @brief  解析剩余位移帧数据
  * @param  data: 8字节CAN数据 (byte[0-3]=主剩余位移, byte[4-7]=从剩余位移，小端，单位0.1mm)
  * @param  main_rem: 输出主剩余位移（单位0.1mm）
  * @param  slave_rem: 输出从剩余位移（单位0.1mm）
  * @retval 无
  */
void CAN_ParseRemainingDisplacement(const uint8_t *data, uint32_t *main_rem, uint32_t *slave_rem)
{
	/* 解析主剩余位移 byte[0-3]（小端） */
	*main_rem = (uint32_t)data[0] |
	            ((uint32_t)data[1] << 8) |
	            ((uint32_t)data[2] << 16) |
	            ((uint32_t)data[3] << 24);
	
	/* 解析从剩余位移 byte[4-7]（小端） */
	*slave_rem = (uint32_t)data[4] |
	             ((uint32_t)data[5] << 8) |
	             ((uint32_t)data[6] << 16) |
	             ((uint32_t)data[7] << 24);
}

/************************ 文件结束 ****/
