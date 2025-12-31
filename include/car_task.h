/**
  ******************************************************************************
  * @file    car_task.h
  * @author  应用团队
  * @brief   小车任务控制头文件
  ******************************************************************************
  */

#ifndef __CAR_TASK_H
#define __CAR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <../CMSIS_RTOS/cmsis_os.h>
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  小车锁定任务：从队列中接收 CAN 帧并处理状态机逻辑
  * @param  argument 未使用
  * @retval 无
  */
void Car_LockTask(void const *argument);

/**
  * @brief  小车测试任务：前进/后退循环测试
  * @param  argument 未使用
  * @retval 无
  */
void Car_TestTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_TASK_H */

/************************ 文件结束 ****/

/* 外部可见的左轮剩余位移 CAN ID（0 表示未设置） */
extern uint16_t left_remaining_id;
