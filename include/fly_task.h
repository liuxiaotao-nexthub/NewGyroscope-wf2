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

/* 电机回零任务句柄 */
extern osThreadId g_motorHomeTaskHandle;

/* 拉取任务句柄 */
extern osThreadId g_pullTaskHandle;

/* 推出任务句柄 */
extern osThreadId g_pushTaskHandle;

/* 拉取完成信号量 */
extern osSemaphoreId g_pullDoneSemHandle;

/* 推出完成信号量 */
extern osSemaphoreId g_pushDoneSemHandle;

/* ========== 任务函数声明 ========== */

/**
  * @brief  电机绑定任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_BindTask(void const *argument);

/**
  * @brief  电机回零任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_HomeTask(void const *argument);

/**
  * @brief  飞箱拉取任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  * @note   拉上箱子流程
  */
void FlyBox_PullTask(void const *argument);

/**
  * @brief  飞箱推出任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  * @note   放下箱子流程
  */
void FlyBox_PushTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __FLY_TASK_H */
