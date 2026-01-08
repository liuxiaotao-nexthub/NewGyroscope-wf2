/**
  ******************************************************************************
  * @file    fly_task.c
  * @brief   飞箱任务模块实现
  ******************************************************************************
  */

#include "fly_task.h"
#include "flybox.h"
#include "motor.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include "queue.h"
#include <string.h>

/* 外部变量（CAN队列句柄，在 NewGyroscope.c 中定义） */
extern osMessageQId CarCanQueueHandle;

/* CAN消息结构（与can_task.c一致） */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} CarCanMsg_t;

/* 当前识别到的飞箱ID（0表示未识别） */
uint8_t g_current_flybox_id = 0;

/* 电机运动参数配置 */
#define MOTOR_WHEEL_DIAMETER    50      /* 轮径 50mm */
#define MOTOR_ACCELERATION      1000    /* 加速度 1000mm/s² */
#define MOTOR_VELOCITY          500     /* 速度 500mm/s */

/* 设备号数组（按MotorIndex_t顺序） */
static const uint8_t g_motor_dev_ids[MOTOR_COUNT] = {
    DEV_LEFT_TRACK,
    DEV_RIGHT_TRACK,
    DEV_BOTTOM_BELT,
    DEV_TURNTABLE,
    DEV_HOOK
};

/* ========== 任务实现 ========== */

/**
  * @brief  电机绑定任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_BindTask(void const *argument)
{
    (void)argument;
    
    CarCanMsg_t carMsg;
    const FlyBoxConfig_t *pConfig = NULL;
    
    /* 等待系统稳定 */
    osDelay(500);
    
    /* ===== 阶段1: 识别飞箱 ===== */
    while (pConfig == NULL)
    {
        Motor_RequestSN();
        
        if (xQueueReceive(CarCanQueueHandle, &carMsg, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (carMsg.id == 0x312 && carMsg.len == 7)
            {
                /* 遍历所有飞箱配置，匹配SN */
                for (int fb = 0; fb < FLYBOX_COUNT; fb++)
                {
                    for (int m = 0; m < MOTOR_COUNT; m++)
                    {
                        if (memcmp(g_flybox_configs[fb].motor_sn[m], carMsg.data, 7) == 0)
                        {
                            pConfig = &g_flybox_configs[fb];
                            g_current_flybox_id = pConfig->id;
                            break;
                        }
                    }
                    if (pConfig != NULL) break;
                }
            }
        }
        osDelay(50);
    }
    
    /* ===== 阶段2: 配置所有电机 ===== */
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        uint8_t dev_id = g_motor_dev_ids[i];
        
        Motor_BindSN(pConfig->motor_sn[i], dev_id);
        osDelay(10);
        
        Motor_InitParams(dev_id);
        osDelay(10);
        
        Motor_Enable(MOTOR_ENABLE, dev_id);
        osDelay(10);
        
        Motor_SetWheelDiameter(dev_id, MOTOR_WHEEL_DIAMETER);
        osDelay(10);
        
        Motor_SetAcceleration(dev_id, MOTOR_ACCELERATION);
        osDelay(10);
        
        Motor_SetVelocity(dev_id, MOTOR_VELOCITY);
        osDelay(10);
    }
    
    /* 绑定完成，任务挂起 */
    vTaskSuspend(NULL);
}
