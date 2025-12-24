/**
  ******************************************************************************
  * @file    PID.c
  * @author  应用团队
  * @brief   PID控制器实现
  *          用于小车直线运动时的角度校正控制
  * 
  * @note    小车参数：
  *          - 实际轮径：77.25mm (代码轮径25.75mm × 3)
  *          - 轮距：382mm (两轮各转600mm → 旋转180°，轮距 = 2×600/π)
  *          - 控制周期：10ms (100Hz)
  * 
  * @note    PID参数设计（保守调整，避免过补偿）：

  *          Kp = 0.5: 1度偏差 → 补偿0.5mm位移（温和修正）
  *          Ki = 0.02: 微弱积分，缓慢消除稳态误差
  *          Kd = 0.1: 小阻尼，避免震荡
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
/* PID控制器参数（可在GDB调试时修改） */
float g_pid_kp = 0.5f;      /* 比例系数：角度偏差1度 -> 补偿0.5mm */
float g_pid_ki = 0.02f;     /* 积分系数：累积误差补偿 */
float g_pid_kd = 0.1f;      /* 微分系数：抑制震荡 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  PID控制器计算
  * @param  state: PID状态结构体指针
  * @param  error: 当前误差（期望角度 - 实际角度）
  * @param  dt: 时间间隔（秒）
  * @retval 控制输出（补偿位移，单位：mm）
  * 
  * @note   输出限幅：
  *         - P项：±15mm (对应±30度误差)
  *         - I项：±5mm (通过积分限幅实现)
  *         - D项：±10mm (对应快速变化)
  *         - 总输出：±20mm
  */
float PID_Calculate(PID_State_t *state, float error, float dt)
{
    /* 比例项 */
    float p_term = g_pid_kp * error;
    
    /* P项限幅（防止单项过大） */
    if (p_term > 15.0f) {
        p_term = 15.0f;
    } else if (p_term < -15.0f) {
        p_term = -15.0f;
    }
    
    /* 积分项（带限幅防止积分饱和） */
    state->integral += error * dt;
    if (state->integral > state->integral_limit) {
        state->integral = state->integral_limit;
    } else if (state->integral < -state->integral_limit) {
        state->integral = -state->integral_limit;
    }
    float i_term = g_pid_ki * state->integral;
    
    /* 微分项 */
    float derivative = (error - state->prev_error) / dt;
    float d_term = g_pid_kd * derivative;
    
    /* D项限幅（防止噪声放大） */
    if (d_term > 10.0f) {
        d_term = 10.0f;
    } else if (d_term < -10.0f) {
        d_term = -10.0f;
    }
    
    /* 更新状态 */
    state->prev_error = error;
    
    /* 总输出（带限幅） */
    float output = p_term + i_term + d_term;
    
    /* 总输出限幅 */
    if (output > 20.0f) {
        output = 20.0f;
    } else if (output < -20.0f) {
        output = -20.0f;
    }
    
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
    state->integral_limit = 30.0f;  /* 积分限幅：±30度·秒 (对应±0.6mm补偿) */
}

/************************ 文件结束 ****/
