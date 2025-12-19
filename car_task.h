/**
  ******************************************************************************
  * @file    car_task.h
  * @author  应用团队
  * @brief   小车控制任务头文件
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
  * @brief  小车锁定任务：从队列接收 CAN 帧并按状态机处理
  * @param  argument 未使用
  * @retval 无
  */
void Car_LockTask(void const *argument);

/**
  * @brief  小车运行测试任务：前进/后退循环测试
  * @param  argument 未使用
  * @retval 无
  */
void Car_TestTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_TASK_H */

/************************ 文件结束 ****/
