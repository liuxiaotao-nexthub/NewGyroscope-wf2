/**
  ******************************************************************************
  * @file    PID.c
  * @author  应用团队
  * @brief   PID控制器实现
  *          用于小车直线运动时的角度校正控制
  * 
  * @note    小车参数：
  *          - 实际轮径：77.25mm (测量轮径25.75mm × 3)
  *          - 轮距：约764mm (经验证：两轮各旋转180° × 各轮位移1200mm)
  *          - 控制周期：5ms (200Hz)
  *          - 直线运动时角度偏差约为0.6度以内
  * 
  * @note    PID参数设计（当前实际运行参数）
  *          Kp = 6.67: 1度偏差 → 产生6.67mm位移 (按1200mm/180度推导)
  *          Ki = 0: 不使用积分，偏差小不需要消除长期静态误差
  *          Kd = 0: 不使用微分，偏差变化慢不需要微分控制
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
/* PID参数（可在GDB调试时修改） */
float g_pid_kp = 7.1f;      /* 比例系数 - 提高到8.0以增强角度校正 */

/* PID输出（调试、监控用，可在GDB中查看） */
float g_pid_output = 0.0f;

/* 内部限幅常量 */
static const float PID_OUTPUT_LIMIT = 20.0f;    /* 输出限幅（mm） */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  PID控制器计算
  * @param  state: PID状态结构体指针
  * @param  error: 当前误差（目标角度 - 实际角度）
  * @param  dt: 时间间隔（秒）
  * @retval 控制输出：差分轮位移（单位：mm）
  * 
  * @note   由于仅P控制：
  *         - 角度误差0.6° → 产生约±4mm
  *         - 输出限幅设为10mm，防止异常大角度偏差时过度控制
  */
float PID_Calculate(PID_State_t *state, float error, float dt)
{
    /* 比例项 */
    float p_term = g_pid_kp * error;
    /* 合成输出 */
    float output = p_term;

    /* 输出限幅 */
    if (output > PID_OUTPUT_LIMIT) output = PID_OUTPUT_LIMIT;
    if (output < -PID_OUTPUT_LIMIT) output = -PID_OUTPUT_LIMIT;

    /* 保存输出供调试 */
    g_pid_output = output;

    return output;
}

/**
  * @brief  重置PID状态
  * @param  state: PID状态结构体指针
  * @retval 无
  */
void PID_Reset(PID_State_t *state)
{
    state->integral = 0.0f;
    state->prev_error = 0.0f;
    state->integral_limit = 0.0f;  /* 不使用积分 */
}

/************************ 文件结束 ****/
