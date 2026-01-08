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

/* ========== 函数声明 ========== */

/**
  * @brief  电机绑定任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_BindTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif /* __FLY_TASK_H */
