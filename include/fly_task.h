/**
  ******************************************************************************
  * @file    fly_task.h
  * @brief   飞箱任务模块头文件
  ******************************************************************************
  */

#ifndef __FLY_TASK_H
#define __FLY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <../CMSIS_RTOS/cmsis_os.h>

/* ========== 外部变量 ========== */

/* 当前识别到的飞箱ID（0表示未识别） */
extern uint8_t g_current_flybox_id;

/* 测试任务句柄 */
extern osThreadId g_testTaskHandle;

/* ========== 任务函数声明 ========== */

/**
  * @brief  电机绑定任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_BindTask(void const *argument);

/**
  * @brief  飞箱测试任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void FlyBox_TestTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __FLY_TASK_H */
