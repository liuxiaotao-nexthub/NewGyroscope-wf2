/**
  ******************************************************************************
  * @file    PID.c
  * @author  应用团队
  * @brief   PID控制器实现
  *          用于小车直线运动时的角度校正控制
  * 
  * @note    小车参数：
  *          - 实际轮径：77.25mm (代码轮径25.75mm × 3)
  *          - 轮距：约764mm (定一边轮子，旋转180° → 电机移动1200mm)
  *          - 控制周期：10ms (100Hz)
  *          - 直线运动时角度偏差：约±0.6°以内
  * 
  * @note    PID参数设计（基于实测参数）：
  *          Kp = 6.67: 1度偏差 → 补偿6.67mm位移 (由1200mm/180°推导)
  *          Ki = 0: 不使用积分（偏差小，不需要积分消除稳态误差）
  *          Kd = 0: 不使用微分（偏差变化慢，不需要微分抑制）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
/* PID控制器参数（可在GDB调试时修改） */
float g_pid_kp = 1.0f;      /* 比例系数：1度 → 6.67mm (1200mm/180°) */
float g_pid_ki = 0.0f;       /* 积分系数：不使用 */
float g_pid_kd = 0.0f;       /* 微分系数：不使用 */

/* PID输出（用于调试、可在GDB中查看） */
float g_pid_output = 0.0f;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  PID控制器计算
  * @param  state: PID状态结构体指针
  * @param  error: 当前误差（期望角度 - 实际角度）
  * @param  dt: 时间间隔（秒）
  * @retval 控制输出（补偿位移，单位：mm）
  * 
  * @note   简化为纯P控制：
  *         - 角度误差±0.6° → 补偿约±4mm
  *         - 输出限幅：±10mm（防止异常大角度偏差时过补偿）
  */
float PID_Calculate(PID_State_t *state, float error, float dt)
{
    /* 纯比例控制（P控制） */
    float output = g_pid_kp * error;
    
    /* 输出限幅（防止异常情况） */
    if (output > 10.0f) {
        output = 10.0f;
    } else if (output < -10.0f) {
        output = -10.0f;
    }
    
    /* 更新状态（虽然不使用I和D，但保持接口一致性） */
    state->prev_error = error;
    
    /* 导出输出供调试 */
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
