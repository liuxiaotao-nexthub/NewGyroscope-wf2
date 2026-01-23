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
#include <math.h>
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

/* 电机回零任务句柄 */
osThreadId g_motorHomeTaskHandle = NULL;

/* 测试任务句柄 */
osThreadId g_testTaskHandle = NULL;

/* 电机运动参数配置 */
#define MOTOR_WHEEL_DIAMETER    50       /* 轮径 50mm */

/* 各电机加速度配置 */
#define ACCELERATION_TRACK      2000     /* 履带加速度 2000mm/s² */
#define ACCELERATION_BELT       12000     /* 底带加速度 12000mm/s² */
#define ACCELERATION_TURNTABLE  12000     /* 转盘加速度 12000mm/s² */
#define ACCELERATION_HOOK       12000     /* 抓钩加速度 12000mm/s² */

/* 各电机速度配置 */
#define VELOCITY_TRACK          2000      /* 履带速度 2000mm/s */
#define VELOCITY_BELT           9700      /* 底带速度 9700mm/s */
#define VELOCITY_TURNTABLE      10000      /* 转盘速度 10000mm/s */
#define VELOCITY_HOOK           10000      /* 抓钩速度 10000mm/s */

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
    
    /* ===== 为 TOF 测距模块 发送配置 ===== */
    {
        uint8_t tof_ids[2] = {0x19, 0x1A};
        uint8_t tof_buf[8] = {0x11, 0x22, 0x00, 0xFF, 0x00, 0x05, 0x00, 0x00};

        for (int t = 0; t < 2; t++) {
            tof_buf[7] = tof_ids[t];
            for (int k = 0; k < 3; k++) {
                CAN_SendData(0x316, tof_buf, 8);
                osDelay(1);
            }
        }
        /* 配置完 0x316 后，启动 0x19/0x1A 的连续测量（CAN ID 0x0408） */
        {
            uint8_t start1[8] = {0x19, 0x0A, 0x01, 0x11, 0x22, 0x00, 0x00, 0x00};
            uint8_t start2[8] = {0x1A, 0x0A, 0x01, 0x11, 0x22, 0x00, 0x00, 0x00};
            for (int k = 0; k < 3; k++) {
                CAN_SendData(0x0408, start1, 8);
                osDelay(1);
            }
            for (int k = 0; k < 3; k++) {
                CAN_SendData(0x0408, start2, 8);
                osDelay(1);
            }
        }
    }
    
    /* 绑定完成，启动电机回零任务 */
    if (g_motorHomeTaskHandle != NULL)
    {
        osThreadResume(g_motorHomeTaskHandle);
    }
    /* 绑定完成，任务挂起 */
	vTaskSuspend(NULL);
}

/**
  * @brief  电机回零任务（占位，待实现）
  * @param  argument: 任务参数（未使用）
  * @retval None
  */
void Motor_HomeTask(void const *argument)
{
    (void)argument;
    
    /* 任务开始时先挂起，等待绑定任务恢夏 */
    vTaskSuspend(NULL);
    
    /* TODO: 实现电机回零流程 */
    /* 回零完成后，恢复测试任务 */
    if (g_testTaskHandle != NULL)
    {
        osThreadResume(g_testTaskHandle);
    }
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
    CarCanMsg_t carMsg;
    
    /* 任务开始时先挂起，等待回零任务恢夏 */
    vTaskSuspend(NULL);
    
    float last_belt_move_mm = 0.0f;

    for (;;)
    {
        /* ============================================== */
        /* ========== 拉上箱子流程 ========== */
        /* ============================================== */
        
        /* ===== 步骤1: 底带前进，准备抓取 ===== */
        /* 使用 TOF 读取距离并移动到底带停在 (x - 80mm)
           每次循环发送请求，若未收到回复则再次询问（最多等待约2秒） */
        {
            const uint8_t tof_dev = 0x19; /* 使用 TOF 设备号 0x19（可改为 0x1A） */
            float dist_mm = 0.0f;
            uint8_t resp_dev = 0;
            uint8_t resp_type = 0;
            int parsed = 0;

            /* 循环发送请求并等待短时响应 */
            for (int wait = 0; wait < 200; wait++) {
                /* 每次循环重新发送请求，如果设备没有回复就再次询问 */
                FlyBox_RequestTOF(tof_dev);

                if (xQueueReceive(CarCanQueueHandle, &carMsg, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (carMsg.id == 0x409 && carMsg.len >= 4 && carMsg.data[0] == tof_dev) {
                        if (FlyBox_ParseTOF(carMsg.data, carMsg.len, &resp_dev, &dist_mm, &resp_type) == 0) {
                            parsed = 1;
                            break;
                        }
                    }
                }
                osDelay(20);
                /* 若未收到，继续下一轮（再次发送请求） */
            }

            if (parsed && dist_mm > 0.0f) {
                /* 目标位置为 x -80mm（8cm），若负则不移动 */
                float move_mm = dist_mm - 80.0f;
                if (move_mm > 0.0f) {
                    /* 限制最大前进为 550mm（55cm）以避免碰撞 */
                    if (move_mm > 550.0f) move_mm = 550.0f;
                    FlyBox_BeltMove(move_mm);
                    /* 记录底带实际前进量，供抓取后回退使用 */
                    last_belt_move_mm = move_mm;

                    /* 继续查询 TOF，直到距离稳定：连续 5 次变化 < 5mm（0.5cm）视为稳定 */
                    float prev_dist = dist_mm;
                    int stable_count = 0;
                    for (int iter = 0; iter < 100; iter++) { /* 最多等待约10秒 */
                        FlyBox_RequestTOF(tof_dev);
                        float new_dist = prev_dist;
	                    if (xQueueReceive(CarCanQueueHandle, &carMsg, pdMS_TO_TICKS(50)) == pdTRUE) {
                            if (carMsg.id == 0x409 && carMsg.len >= 4 && carMsg.data[0] == tof_dev) {
                                uint8_t tmp_dev, tmp_type;
                                if (FlyBox_ParseTOF(carMsg.data, carMsg.len, &tmp_dev, &new_dist, &tmp_type) == 0) {
                                    /* parsed new_dist */
                                }
                            }
	                    }

                        /* 判断变化是否小于阈值 (5 mm) */
                        if (fabsf(new_dist - prev_dist) < 5.0f) {
                            stable_count++;
                        } else {
                            stable_count = 0;
                        }
                        prev_dist = new_dist;

                        if (stable_count >= 3) {
                            break; /* 稳定，继续后续步骤 */
                        }
                        osDelay(25); /* 等待一段时间再查询 */
                    }
                }
            }
        }
        
        /* ===== 步骤2: 抓钩抓取（上钩90°） ===== */
        FlyBox_HookRotate(-90.0f);       /* 抓钩上钩90° */
        osDelay(600);                   /* 等待抓取完成 */
        
        /* ===== 步骤3/4: 抓取后，按当前底带位置退回到位置 0cm，并让履带做相同位移 ===== */
        {
            /* 计算需要后退的距离（mm）使底带位置变为 0：全局以 mm 存储 */
            float retreat_mm = g_belt_position_mm; /* mm */
            if (retreat_mm > 0.0f) {
                /* 发送后退指令 */
                FlyBox_BeltMove(-retreat_mm);
                FlyBox_TrackMove(-retreat_mm);

                /* 等待并检测底带电机位移的稳定状态：通过请求电机位移并解析，
                   若连续 5 次读数变化在 2mm 以内则认为静止 */
                const int stable_needed = 5;
                int stable_count = 0;
                float last_pos = 0.0f;
                int has_last = 0;

                for (int attempt = 0; attempt < 200; attempt++) { /* 上限迭代，避免死循环 */
                    /* 请求电机返回位置 */
                    Motor_RequestPosition(DEV_BOTTOM_BELT);

                    if (xQueueReceive(CarCanQueueHandle, &carMsg, pdMS_TO_TICKS(50)) == pdTRUE) {
                        if (carMsg.id == 0x409 && carMsg.len >= 6) {
                            uint8_t resp_dev = 0;
                            float pos_mm = Motor_ParsePosition(carMsg.data, &resp_dev);
                            if (resp_dev == DEV_BOTTOM_BELT) {
                                if (!has_last) {
                                    last_pos = pos_mm;
                                    has_last = 1;
                                    stable_count = 1;
                                } else {
                                    if (fabsf(pos_mm - last_pos) <= 2.0f) {
                                        stable_count++;
                                    } else {
                                        stable_count = 1;
                                        last_pos = pos_mm;
                                    }
                                }

                                if (stable_count >= stable_needed) {
                                    break; /* 稳定，后退完成 */
                                }
                            }
                        }
                    }

                    osDelay(25);
                }

                /* 全局位置由其他发送位移的代码维护，此处不再修改 g_belt_position_mm */
            }
        }
        
        /* ===== 步骤5: 抓钩放下（下钩90°）+转盘复位 ===== */
        FlyBox_HookRotate(90.0f);        /* 抓钩下钩90° */
        osDelay(600);
        FlyBox_TurntableRotate(90.0f);  /* 转盘逆时针90° */
        osDelay(50);
        /* 履带后退15cm */
        FlyBox_TrackMove(-150.0f);      /* 两边履带后退150mm */
        /* 等待履带移动完成：连续5次变化<5mm */
        {
            const int stable_needed = 5;
            int stable_count = 0;
            float last_pos = 0.0f;
            int has_last = 0;
            CarCanMsg_t track_msg;
            for (int attempt = 0; attempt < 200; attempt++) {
                Motor_RequestPosition(DEV_LEFT_TRACK);
                if (xQueueReceive(CarCanQueueHandle, &track_msg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (track_msg.id == 0x409 && track_msg.len >= 6) {
                        uint8_t resp_dev = 0;
                        float pos_mm = Motor_ParsePosition(track_msg.data, &resp_dev);
                        if (resp_dev == DEV_LEFT_TRACK) {
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) < 5.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }
                            if (stable_count >= stable_needed) break;
                        }
                    }
                }
                osDelay(25);
            }
        }
        /* 等待转盘动作完成：连续5次变化<5mm */
        {
            const int stable_needed = 5;
            int stable_count = 0;
            float last_pos = 0.0f;
            int has_last = 0;
            CarCanMsg_t turn_msg;
            for (int attempt = 0; attempt < 200; attempt++) {
                Motor_RequestPosition(DEV_TURNTABLE);
                if (xQueueReceive(CarCanQueueHandle, &turn_msg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (turn_msg.id == 0x409 && turn_msg.len >= 6) {
                        uint8_t resp_dev = 0;
                        float pos_mm = Motor_ParsePosition(turn_msg.data, &resp_dev);
                        if (resp_dev == DEV_TURNTABLE) {
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) < 5.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }
                            if (stable_count >= stable_needed) break;
                        }
                    }
                }
                osDelay(25);
            }
        }
        
        /* ===== 步骤6: 使用 TOF(0x1A) 驱动履带后退，直到箱子距离 < 8.5cm =====
           流程：
           1) 请求 TOF(0x1A) 得到当前距离 x (mm)
           2) 计算目标位置 x-80mm，向后移动履带使箱子靠近（如果需要）
           3) 等待履带电机稳定：连续 5 次读数变化在 2mm 内
           4) 读取 TOF，若距离 < 85mm 则退出，否则重复整个流程（while 循环）
        */
        {
            const uint8_t tof_dev = 0x1A;
            CarCanMsg_t resp;

            for (;;) {
                /* 1) 请求 TOF 并解析 */
                float dist_mm = 0.0f;
                uint8_t rdev = 0, rtype = 0;
                int parsed = 0;

                for (int i = 0; i < 200; i++) {
                    FlyBox_RequestTOF(tof_dev);
                    if (xQueueReceive(CarCanQueueHandle, &resp, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (resp.id == 0x409 && resp.len >= 5 && resp.data[0] == tof_dev) {
                            if (FlyBox_ParseTOF(resp.data, resp.len, &rdev, &dist_mm, &rtype) == 0) {
                                parsed = 1;
                                break;
                            }
                        }
                    }
                    osDelay(20);
                }


                /* 2) 计算目标并移动，目标为 4~7cm 范围：正为前进，负为后退 */
                /* 如果当前距离在 40~70mm（4~7cm）范围内，不移动 */
                if (dist_mm >= 40.0f && dist_mm <= 70.0f) {
                    break; /* 已在目标范围内，跳出循环 */
                }
                /* 否则计算移动距离，目标为 55mm（5.5cm，范围中点） */
                float move_mm = 55.0f - dist_mm;
                FlyBox_TrackMove(move_mm);

                /* 3) 等待履带电机稳定：连续 5 次读数变化在 2mm 内 */
                const int stable_needed = 5;
                int stable_count = 0;
                float last_pos = 0.0f;
                int has_last = 0;

                for (int attempt = 0; attempt < 200; attempt++) {
                    Motor_RequestPosition(DEV_LEFT_TRACK);
                    if (xQueueReceive(CarCanQueueHandle, &resp, pdMS_TO_TICKS(150)) == pdTRUE) {
                        if (resp.id == 0x409 && resp.len >= 6 && resp.data[0] == DEV_LEFT_TRACK) {
                            uint8_t rdev2 = 0;
                            float pos_mm = Motor_ParsePosition(resp.data, &rdev2);
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) <= 5.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }

                            if (stable_count >= stable_needed) {
                                break; /* 履带稳定 */
                            }
                        }
                    }
                    osDelay(25);
                }

                /* 4) 读取 TOF 确认距离（读到数据就退出，由外层判断是否满足条件） */
                float current_dist = dist_mm;
                for (int j = 0; j < 200; j++) {
                    FlyBox_RequestTOF(tof_dev);
                    if (xQueueReceive(CarCanQueueHandle, &resp, pdMS_TO_TICKS(100)) == pdTRUE) {
                        if (resp.id == 0x409 && resp.len >= 5 && resp.data[0] == tof_dev) {
                            uint8_t tmpd = 0, tmpt = 0;
                            if (FlyBox_ParseTOF(resp.data, resp.len, &tmpd, &current_dist, &tmpt) == 0) {
                                break; /* 读到数据就退出 */
                            }
                        }
                    }
                    osDelay(20);
                }

                /* 如果当前距离在 40~70mm（4~7cm）范围内，跳出循环，继续后续步骤 */
                if (current_dist >= 40.0f && current_dist <= 70.0f) break;
                /* 否则继续循环重新计算移动 */
            }
        }
        
        /* 拉上完成，等待一段时间 */
        osDelay(1000);

        /* 重置记录，准备下一个周期 */
        last_belt_move_mm = 0.0f;
        
        /* ============================================== */
        /* ========== 放下箱子流程（反向操作） ========== */
        /* ============================================== */
        
        /* ===== 步骤1: 使用 TOF(0x1A) 驱动履带前进，准备送出箱子（循环直到 TOF 在 23~24cm 范围内） ===== */
        {
            const uint8_t tof_dev = 0x1A;
            CarCanMsg_t resp;

            for (;;) {
                /* 请求并读取 TOF 值 */
                float dist_mm = 0.0f;
                uint8_t rdev = 0, rtype = 0;
                int parsed = 0;

                for (int i = 0; i < 200; i++) {
                    FlyBox_RequestTOF(tof_dev);
                    if (xQueueReceive(CarCanQueueHandle, &resp, pdMS_TO_TICKS(50)) == pdTRUE) {
                        if (resp.id == 0x409 && resp.len >= 5 && resp.data[0] == tof_dev) {
                            if (FlyBox_ParseTOF(resp.data, resp.len, &rdev, &dist_mm, &rtype) == 0) {
                                parsed = 1;
                                break;
                            }
                        }
                    }
                    osDelay(20);
                }

                /* 判断距离是否在目标范围内 */
                if (parsed && dist_mm >= 200.0f && dist_mm < 210.0f) {
                    break; /* 距离在 20~21cm 范围内，退出循环 */
                }

                /* 计算目标并移动，目标为 200mm（20cm）：正为前进，负为后退 */
                float move_mm = 200.0f - dist_mm;
                FlyBox_TrackMove(move_mm);

                /* 等待履带稳定（连续3次变化<=5mm） */
                const int stable_needed = 3;
                int stable_count = 0;
                float last_pos = 0.0f;
                int has_last = 0;

                for (int attempt = 0; attempt < 200; attempt++) {
                    Motor_RequestPosition(DEV_LEFT_TRACK);
                    if (xQueueReceive(CarCanQueueHandle, &resp, pdMS_TO_TICKS(50)) == pdTRUE) {
                        if (resp.id == 0x409 && resp.len >= 6 && resp.data[0] == DEV_LEFT_TRACK) {
                            uint8_t rdev2 = 0;
                            float pos_mm = Motor_ParsePosition(resp.data, &rdev2);
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) <= 5.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }

                            if (stable_count >= stable_needed) break; /* 履带稳定 */
                        }
                    }
                    osDelay(25);
                }

                /* 继续下一轮读取/移动 */
            }
        }
        
        /* ===== 步骤2: 转盘旋转，转完后抓钩抓取 ===== */
        FlyBox_TurntableRotate(-90.0f);   /* 转盘顺时针90° */
        osDelay(50);
        /* 等待转盘动作完成：连续5次变化<5mm */
        {
            const int stable_needed = 5;
            int stable_count = 0;
            float last_pos = 0.0f;
            int has_last = 0;
            CarCanMsg_t turn_msg;
            for (int attempt = 0; attempt < 200; attempt++) {
                Motor_RequestPosition(DEV_TURNTABLE);
                if (xQueueReceive(CarCanQueueHandle, &turn_msg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (turn_msg.id == 0x409 && turn_msg.len >= 6) {
                        uint8_t resp_dev = 0;
                        float pos_mm = Motor_ParsePosition(turn_msg.data, &resp_dev);
                        if (resp_dev == DEV_TURNTABLE) {
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) < 5.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }
                            if (stable_count >= stable_needed) break;
                        }
                    }
                }
                osDelay(25);
            }
        }
        FlyBox_HookRotate(-90.0f);       /* 抓钩上钩90° */
        osDelay(800);                   /* 等待抓取完成 */
        
        /* ===== 步骤3: 履带和底带同步前进，传送箱子 ===== */
        FlyBox_BeltMove(550.0f);         /* 底带前进55cm */
        osDelay(1);
        FlyBox_TrackMove(550.0f);        /* 履带前进55cm */
        osDelay(1);

        /* 等待底带电机稳定：连续5次读数变化<=2mm视为移动完成 */
        {
            const int stable_needed = 5;
            int stable_count = 0;
            float last_pos = 0.0f;
            int has_last = 0;

            for (int attempt = 0; attempt < 200; attempt++) {
                Motor_RequestPosition(DEV_BOTTOM_BELT);
                if (xQueueReceive(CarCanQueueHandle, &carMsg, pdMS_TO_TICKS(50)) == pdTRUE) {
                    if (carMsg.id == 0x409 && carMsg.len >= 6) {
                        uint8_t resp_dev = 0;
                        float pos_mm = Motor_ParsePosition(carMsg.data, &resp_dev);
                        if (resp_dev == DEV_BOTTOM_BELT) {
                            if (!has_last) {
                                last_pos = pos_mm;
                                has_last = 1;
                                stable_count = 1;
                            } else {
                                if (fabsf(pos_mm - last_pos) <= 2.0f) {
                                    stable_count++;
                                } else {
                                    stable_count = 1;
                                    last_pos = pos_mm;
                                }
                            }

                            if (stable_count >= stable_needed) {
                                break; /* 底带稳定，移动完成 */
                            }
                        }
                    }
                }
                osDelay(25);
            }
        }
        
        /* ===== 步骤5: 只放下抓钩（下钩90°），不回默认位置 ===== */
        FlyBox_HookRotate(90.0f);        /* 抓钩下钩90° */
        osDelay(800);
        /* 放下完成，等待下一个周期 */
        osDelay(5000);
    }
}
