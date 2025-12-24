/**
  ******************************************************************************
  * @file    PID.h
  * @author  应用团队
  * @brief   PID控制器头文件
  *          用于小车直线运动时的角度校正控制
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  PID状态结构体
  */
typedef struct {
    float integral;         /* 积分累积 */
    float prev_error;       /* 上次误差 */
    float integral_limit;   /* 积分限幅 */
} PID_State_t;

/* Exported variables --------------------------------------------------------*/
/* PID控制器参数（可在GDB调试时修改） */
extern float g_pid_kp;      /* 比例系数 */
extern float g_pid_ki;      /* 积分系数 */
extern float g_pid_kd;      /* 微分系数 */

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  PID控制器计算
  * @param  state: PID状态结构体指针
  * @param  error: 当前误差（期望值 - 实际值）
  * @param  dt: 时间间隔（秒）
  * @retval 控制输出（补偿量）
  */
float PID_Calculate(PID_State_t *state, float error, float dt);

/**
  * @brief  重置PID状态
  * @param  state: PID状态结构体指针
  * @retval 无
  */
void PID_Reset(PID_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */

/************************ 文件结束 ****/
