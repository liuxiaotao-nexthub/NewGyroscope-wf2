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

/* 测试任务句柄 */
osThreadId g_testTaskHandle = NULL;

/* 电机运动参数配置 */
#define MOTOR_WHEEL_DIAMETER    500      /* 轮径 50mm */

/* 各电机加速度配置 */
#define ACCELERATION_TRACK      30000    /* 履带加速度 10000mm/s² */
#define ACCELERATION_BELT       50000    /* 底带加速度 60000mm/s² */
#define ACCELERATION_TURNTABLE  50000    /* 转盘加速度 60000mm/s² */
#define ACCELERATION_HOOK       50000    /* 抓钩加速度 60000mm/s² */

/* 各电机速度配置 */
#define VELOCITY_TRACK          5000     /* 履带速度 5000mm/s */
#define VELOCITY_BELT           30000    /* 底带速度 30000mm/s */
#define VELOCITY_TURNTABLE      20000    /* 转盘速度 10000mm/s */
#define VELOCITY_HOOK           10000    /* 抓钩速度 10000mm/s */

/* 各电机速度数组（按MotorIndex_t顺序） */
static const uint32_t g_motor_velocities[MOTOR_COUNT] = {
    VELOCITY_TRACK,      /* 左履带 */
    VELOCITY_TRACK,      /* 右履带 */
    VELOCITY_BELT,       /* 底带 */
    VELOCITY_TURNTABLE,  /* 转盘 */
    VELOCITY_HOOK        /* 抓钩 */
};

/* 各电机加速度数组（按MotorIndex_t顺序） */
static const uint32_t g_motor_accelerations[MOTOR_COUNT] = {
    ACCELERATION_TRACK,      /* 左履带 */
    ACCELERATION_TRACK,      /* 右履带 */
    ACCELERATION_BELT,       /* 底带 */
    ACCELERATION_TURNTABLE,  /* 转盘 */
    ACCELERATION_HOOK        /* 抓钩 */
};

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
        osDelay(5);
        
        Motor_InitParams(dev_id);
        osDelay(5);
        
        Motor_Enable(MOTOR_ENABLE, dev_id);
        osDelay(5);
        
        Motor_SetWheelDiameter(dev_id, MOTOR_WHEEL_DIAMETER);
        osDelay(5);
        
        Motor_SetAcceleration(dev_id, g_motor_accelerations[i]);
        osDelay(5);
        
        Motor_SetVelocity(dev_id, g_motor_velocities[i]);
        osDelay(5);
    }
    
    /* 绑定完成，恢复测试任务 */
    if (g_testTaskHandle != NULL)
    {
        osThreadResume(g_testTaskHandle);
    }
    
    /* 绑定完成，任务挂起 */
    vTaskSuspend(NULL);
}

/**
  * @brief  飞箱测试任务
  * @param  argument: 任务参数（未使用）
  * @retval None
  * @note   箱子抓取和传送流程测试（拉上+放下循环）
  */
void FlyBox_TestTask(void const *argument)
{
    (void)argument;
    
    /* 任务开始时先挂起，等待绑定任务恢复 */
    vTaskSuspend(NULL);
    
    for (;;)
    {
        /* ============================================== */
        /* ========== 拉上箱子流程 ========== */
        /* ============================================== */
        
        /* ===== 步骤1: 转盘旋转+底带前进，准备抓取 ===== */
        FlyBox_TurntableRotate(90.0f);   /* 转盘顺时针90° */
        osDelay(1);
        FlyBox_BeltMove(550.0f);         /* 底带前进55cm */
        osDelay(3000);                   /* 等待动作完成 */
        
        /* ===== 步骤2: 抓钩抓取 ===== */
        FlyBox_HookGrab();               /* 抓钩抓取 */
        osDelay(1000);                   /* 等待抓取完成 */
        
        /* ===== 步骤3: 底带后退一点，让箱子部分上履带 ===== */
        FlyBox_BeltMove(-100.0f);        /* 底带后退10cm */
        osDelay(1000);                   /* 等待动作完成 */
        
        /* ===== 步骤4: 履带和底带同步后退，传送箱子 ===== */
        FlyBox_BeltMove(-450.0f);        /* 底带后退45cm */
        osDelay(1);
        FlyBox_TrackMove(-450.0f);       /* 履带后退45cm */
        osDelay(3000);                   /* 等待动作完成 */
        
        /* ===== 步骤5: 抓钩放下+转盘复位 ===== */
        FlyBox_HookRelease();            /* 抓钩放下 */
        osDelay(1000);
        FlyBox_TurntableRotate(-90.0f);  /* 转盘逆时针90° */
        osDelay(2000);                   /* 等待动作完成 */
        
        /* ===== 步骤6: 履带后退，让箱子完全上来 ===== */
        FlyBox_TrackMove(-100.0f);       /* 履带后退10cm */
        osDelay(2000);                   /* 等待动作完成 */
        
        /* 拉上完成，等待一段时间 */
        osDelay(3000);
        
        /* ============================================== */
        /* ========== 放下箱子流程（反向操作） ========== */
        /* ============================================== */
        
        /* ===== 步骤1: 履带前进，准备送出箱子 ===== */
        FlyBox_TrackMove(100.0f);        /* 履带前进10cm */
        osDelay(2000);                   /* 等待动作完成 */
        
        /* ===== 步骤2: 转盘旋转，转完后抓钩抓取 ===== */
        FlyBox_TurntableRotate(90.0f);   /* 转盘顺时针90° */
        osDelay(2000);                   /* 等待转盘转完 */
        FlyBox_HookGrab();               /* 抓钩抓取 */
        osDelay(1000);                   /* 等待抓取完成 */
        
        /* ===== 步骤3: 履带和底带同步前进，传送箱子 ===== */
        FlyBox_BeltMove(450.0f);         /* 底带前进45cm */
        osDelay(1);
        FlyBox_TrackMove(450.0f);        /* 履带前进45cm */
        osDelay(3000);                   /* 等待动作完成 */
        
        /* ===== 步骤4: 底带单独前进，箱子到位 ===== */
        FlyBox_BeltMove(100.0f);         /* 底带前进10cm */
        osDelay(1000);                   /* 等待动作完成 */
        
        /* ===== 步骤5: 抓钩放下+转盘复位+底带前进 ===== */
        FlyBox_HookRelease();            /* 抓钩放下 */
        osDelay(1000);
        FlyBox_TurntableRotate(-90.0f);  /* 转盘逆时针90° */
        osDelay(1);
        FlyBox_BeltMove(-550.0f);         /* 底带前进55cm */
        osDelay(1000);                   /* 等待动作完成 */
        
        /* 放下完成，等待下一个周期 */
        osDelay(5000);
    }
}
