/**
  ******************************************************************************
  * @file    car_task.c
  * @author  应用团队
  * @brief   小车控制任务实现
  *          包含小车绑定、初始化、校准和使能等状态机逻辑
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "car_task.h"
#include "car.h"
#include "can.h"
#include "tim.h"
#include "PID.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>
#include <math.h>

/* External variables --------------------------------------------------------*/
extern osMessageQId CarCanQueueHandle;

/* Private typedef -----------------------------------------------------------*/
/* 小车队列消息结构 */
typedef struct {
    uint32_t id;       /* CAN 消息 ID */
    uint8_t data[8];   /* CAN 数据（最多 8 字节） */
    uint8_t len;       /* 数据长度 */
} CarCanMsg_t;

/* 小车状态机状态定义 */
typedef enum {
    CAR_STATE_IDLE = 0,         /* 空闲状态（未使用） */
    CAR_STATE_BINDING_SN,       /* 绑定 SN 状态：等待接收左右轮 SN（ID 0x312）并发送绑定命令（ID 0x313） */
    CAR_STATE_INIT_MOTOR,       /* 电机初始化状态：发送电机参数初始化命令（ID 0x316） */
    CAR_STATE_CALIBRATE_WHEEL,  /* 轮子校准状态：读取电机位移并判断左右轮 */
    CAR_STATE_ENABLE_POWER,     /* 上电使能状态：发送电源使能命令（ID 0x413）并发送锁定命令（ID 0x60D） */
    CAR_STATE_DONE              /* 完成状态：任务自挂起 */
} CarState_t;

/* Private variables ---------------------------------------------------------*/
/* 运动参数全局变量（可在GDB调试时修改） */
float g_linear_velocity = 1500.0f;      /* 直线运动速度 (mm/s) */
float g_rotation_velocity = 1500.0f;     /* 旋转速度 (mm/s) */
float g_acceleration = 1000.0f;         /* 加速度 (mm/s²) */
float g_test_distance = 5000.0f;         /* 测试距离 (mm) */
float g_turn_distance = 613.0f;         /* 旋转位移 (mm) - 两轮各600mm刚好180° */
float g_target_angle = 180.0f;          /* 目标旋转角度 (度) */

/* 全局角度误差，便于调试/查看 */
float g_angle_error = 0.0f;

/* 测试循环计数器 */
uint16_t num_i = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  小车锁定任务：从队列接收 CAN 帧并按状态机处理
  * @param  argument 未使用
  * @retval 无
  */
void Car_LockTask(void const *argument)
{
    (void)argument;

    CarState_t car_state = CAR_STATE_BINDING_SN;

    /* 存储左右轮 SN，每个 SN 7 字节 */
    uint8_t sn_left[7] = {0};
    uint8_t sn_right[7] = {0};
    int have_left = 0;
    int have_right = 0;

    osEvent evt;

    for (;;)
    {
        switch (car_state)
        {
            case CAR_STATE_BINDING_SN:
                /* 状态说明：等待通过队列接收 ID=0x312 的 7 字节 SN 上报，
                   先到的为左轮（绑定为设备号 0x01），随后为右轮（绑定为设备号 0x02）。
                   收到每个 SN 后发送绑定命令（ID 0x313，7 字节 SN + 1 字节设备号）。
                   如果 100ms 内未收到数据，主动发送查询命令（ID 0x60D，数据 07 00）。
                */
                evt = osMessageGet(CarCanQueueHandle, 100); /* 等待 100ms */
                if (evt.status == osEventMessage)
                {
                    CarCanMsg_t *pMsg = (CarCanMsg_t *)evt.value.p;
                    if (pMsg->id == 0x312 && pMsg->len == 7)
                    {
                        if (!have_left)
                        {
                            memcpy(sn_left, pMsg->data, 7);
                            have_left = 1;
                            /* 发送左轮绑定命令（连续发送3遍） */
                            uint8_t buf[8];
                            memcpy(buf, sn_left, 7);
                            buf[7] = CAR_DEV_LEFT;
                            for (int i = 0; i < 3; i++)
                            {
                                CAN_SendData(0x313, buf, 8);
                                osDelay(2);
                            }
                        }
                        else if (!have_right)
                        {
                            /* 判断接收到的 SN 与左轮 SN 是否不同 */
                            if (memcmp(sn_left, pMsg->data, 7) != 0)
                            {
                                memcpy(sn_right, pMsg->data, 7);
                                have_right = 1;
                                /* 发送右轮绑定命令（连续发送3遍） */
                                uint8_t buf[8];
                                memcpy(buf, sn_right, 7);
                                buf[7] = CAR_DEV_RIGHT;
                                for (int i = 0; i < 3; i++)
                                {
                                    CAN_SendData(0x313, buf, 8);
                                    osDelay(2);
                                }
                            }
                            /* 如果 SN 相同，忽略此消息，继续等待 */
                        }

                        if (have_left && have_right)
                        {
                            car_state = CAR_STATE_INIT_MOTOR; /* 进入电机初始化状态 */
                        }
                    }
                }
                else if (evt.status == osEventTimeout)
                {
                    /* 100ms 超时未收到数据，主动发送查询命令 */
                    uint8_t queryCmd[2] = {0x07, 0x00};
                    CAN_SendData(0x60D, queryCmd, 2);
                }
                break;

            case CAR_STATE_INIT_MOTOR:
                /* 状态说明：发送电机初始化参数（ID 0x316），格式为
                   B4 BF CF AA 00 5F DEV DEV（共 8 字节），分别初始化左、右电机。
                   然后设置轮径、加速度和速度
                */
                {
                    /* 1. 发送电机初始化参数 */
                    uint8_t buf_left[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, CAR_DEV_LEFT, CAR_DEV_LEFT};
                    CAN_SendData(0x316, buf_left, 8);
                    osDelay(2);

                    uint8_t buf_right[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, CAR_DEV_RIGHT, CAR_DEV_RIGHT};
                    CAN_SendData(0x316, buf_right, 8);
                    osDelay(2);

                    /* 2. 设置轮径为 25.75mm */
                    Car_SetWheelDiameter(CAR_DEV_LEFT, 2575);   /* 2575 = 25.75mm / 0.01mm */
                    Car_SetWheelDiameter(CAR_DEV_RIGHT, 2575);

                    /* 3. 设置加速度为 g_acceleration（以 0.1 单位发送） */
                    Car_SetAcceleration(CAR_DEV_LEFT, (uint32_t)(g_acceleration * 10.0f));
                    Car_SetAcceleration(CAR_DEV_RIGHT, (uint32_t)(g_acceleration * 10.0f));

                    /* 4. 设置速度为 g_linear_velocity（以 0.1mm/s 单位发送） */
                    Car_SetVelocity(CAR_DEV_LEFT, (uint32_t)(g_linear_velocity * 10.0f));
                    Car_SetVelocity(CAR_DEV_RIGHT, (uint32_t)(g_linear_velocity * 10.0f));

                    /* 5. 设置从加速度为 g_acceleration（以 0.1 单位发送） */
                    Car_SetSlaveAcceleration(CAR_DEV_LEFT, (uint32_t)(g_acceleration * 20.0f));
                    Car_SetSlaveAcceleration(CAR_DEV_RIGHT, (uint32_t)(g_acceleration * 20.0f));

                    /* 6. 设置从速度为 g_linear_velocity（以 0.1mm/s 单位发送） */
                    Car_SetSlaveVelocity(CAR_DEV_LEFT, (uint32_t)(g_linear_velocity * 30.0f));
                    Car_SetSlaveVelocity(CAR_DEV_RIGHT, (uint32_t)(g_linear_velocity * 30.0f));

                    car_state = CAR_STATE_CALIBRATE_WHEEL; /* 跳转到轮子校准状态 */
                }
                break;

            case CAR_STATE_CALIBRATE_WHEEL:
                /* 状态说明：读取电机位移并判断左右轮
                   发送 ID 0x408 读取位移，接收 ID 0x409 的8字节回复
                   等待外力推动小车，当左电机位移变化 >100mm 时：
                   - 位移减小的是左轮
                   - 位移增大的是右轮（说明绑定错误，需要交换）
                   只通过判断左轮变化来决定是否交换设备号
                */
                {
                    static float base_pos_left = 0.0f;  /* 左电机基准位移 */
                    static int got_left_base = 0;

                    /* 首次进入：读取左电机的基准位移 */
                    if (!got_left_base)
                    {
                        Car_ReadMotorPosition(CAR_DEV_LEFT);

                        /* 等待接收基准位移 */
                        evt = osMessageGet(CarCanQueueHandle, 100);
                        if (evt.status == osEventMessage)
                        {
                            CarCanMsg_t *pMsg = (CarCanMsg_t *)evt.value.p;
                            if (pMsg->id == 0x409)
                            {
                                uint8_t dev_id;
                                float pos = Car_ParseMotorPosition(pMsg->data, &dev_id);
                                if (dev_id == CAR_DEV_LEFT)
                                {
                                    base_pos_left = pos;
                                    got_left_base = 1;
                                }
                            }
                        }
                    }
                    else
                    {
                        /* 已获取基准位移，持续读取当前位移判断变化 */
                        Car_ReadMotorPosition(CAR_DEV_LEFT);

                        evt = osMessageGet(CarCanQueueHandle, 100);
                        if (evt.status == osEventMessage)
                        {
                            CarCanMsg_t *pMsg = (CarCanMsg_t *)evt.value.p;
                            if (pMsg->id == 0x409 && pMsg->len >= 6)
                            {
                                uint8_t dev_id;
                                float pos = Car_ParseMotorPosition(pMsg->data, &dev_id);

                                if (dev_id == CAR_DEV_LEFT)
                                {
                                    float delta_left = pos - base_pos_left;
                                    float abs_delta_left = (delta_left < 0) ? -delta_left : delta_left;

                                    if (abs_delta_left > 100.0f)
                                    {
                                        /* 若左电机位移增大（delta>0），说明当前绑定的左设备实际为右轮，需要交换 */
                                        if (delta_left > 0.0f)
                                        {
                                            Car_SwapDeviceID();
                                        }

                                        /* 重置并进入下一状态 */
                                        got_left_base = 0;
                                        base_pos_left = 0.0f;

                                        car_state = CAR_STATE_ENABLE_POWER;
                                    }
                                }
                            }
                        }
                    }
                }
                break;

            case CAR_STATE_ENABLE_POWER:
                /* 状态说明：对左右电机发送上电使能命令（ID 0x413），数据为
                   5A 00 00 00 DEV（使能），发送后再发送锁定命令（ID 0x60D，07 00）。
                */
                {
                    uint8_t left_dev = Car_GetLeftDevID();
                    uint8_t right_dev = Car_GetRightDevID();
                    
                    uint8_t ena_left[5] = {0x5A, 0x00, 0x00, 0x00, left_dev};
                    for (int i = 0; i < 3; i++) {
                        CAN_SendData(0x413, ena_left, 5);
                        osDelay(1);
                    }

                    uint8_t ena_right[5] = {0x5A, 0x00, 0x00, 0x00, right_dev};
                    for (int i = 0; i < 3; i++) {
                        CAN_SendData(0x413, ena_right, 5);
                        osDelay(1);
                    }

                    uint8_t lock[2] = {0x07, 0x00};
                    CAN_SendData(0x60D, lock, 2);

                    car_state = CAR_STATE_DONE; /* 完成后转到 DONE 状态 */
                }
                break;

            case CAR_STATE_DONE:
                /* 状态说明：完成绑定、初始化与上电使能并发送锁定命令后，解挂测试任务并自挂起 */
                {
	                osDelay(3000);
                    /* 解挂测试任务 */
                    if (CarTestTaskHandle != NULL)
                    {
                        vTaskResume(CarTestTaskHandle);
                    }
                    
                    /* 锁定任务自挂起 */
                    vTaskSuspend(NULL);
                }
                break;

            default:
                break;
        }

        osDelay(10);
    }
}

/**
  * @brief  小车运行测试任务：前进 -> 后退 往复
  * @param  argument 未使用
  * @retval 无
  * @note   使用全局变量 g_test_distance，使用角度PID控制保持直线
  */
void Car_TestTask(void const *argument)
{
    (void)argument;

    /* 任务启动时立即挂起，等待锁定任务解挂 */
    vTaskSuspend(NULL);

     /* 每次直走前读取一次角度作为直走基准(base_angle)，旋转校准基准使用 base_angle - 180°（归一化） */

    /* 基准角变量，直走开始时读取并在旋转阶段使用 */
    float base_angle = 0.0f;

    for (; num_i < 10; num_i++)
    {
        /* 前进 g_test_distance - 使用角度PID控制保持直线 */
        {
            uint8_t left_dev = Car_GetLeftDevID();
            uint8_t right_dev = Car_GetRightDevID();
            
            /* 1. 读取初始基准角度 */
            /* 每次直走开始时读取当前角度作为基准 */
            base_angle = TIM_GetAngle();
            if (base_angle > 180.0f) { base_angle -= 360.0f; }
            if (base_angle < -180.0f) { base_angle += 360.0f; }
            // base_angle = 0;
            
            /* 2. 初始化PID控制器 */
            PID_State_t pid_state;
            PID_Reset(&pid_state);
            
            /* 3. 发送前进命令 */
            Car_MoveForward(g_test_distance);
            
            /* 4. 循环读取角度并PID校正，每5ms一次 */
            uint32_t elapsed = 0;
            const uint32_t max_time = 6000;  /* 最多等待8秒 */
            const float dt = 0.01f;  /* 10ms = 0.01秒 */
            int slave_cleared = 0;  /* 标志位：是否已清除从位移 */
            
            while (elapsed < max_time) {
                osDelay(50);
                elapsed += 50;
                
                /* 读取当前角度 */
                float current_angle = TIM_GetAngle();
                
                /* 计算角度误差（期望角度 - 实际角度） */
                /* 目标是保持基准角度不变 */
                float angle_error = base_angle - current_angle;
                
                /* 处理角度跨越±180度的情况 */
                if (angle_error > 180.0f) {
                    angle_error -= 360.0f;
                } else if (angle_error < -180.0f) {
                    angle_error += 360.0f;
                }

                /* 导出到全局，便于调试查看 */
                g_angle_error = angle_error;

                /* PID计算补偿量 */
               float compensation = PID_Calculate(&pid_state, angle_error, dt);
               
               /* 左右轮差分补偿以保持直线 */
               /* angle_error > 0: 顺时针偏转 → 右轮减速、左轮加速 */
               /* angle_error < 0: 逆时针偏转 → 右轮加速、左轮减速 */
                if (fabsf(compensation) > 0.1f) {  /* 补偿阈值0.05mm - 更精细的控制 */
                   /* 补偿量的一半分配给每个轮子，方向相反 */
                   float half_comp = compensation / 1.0f;
                   /* 仅调整左轮 */
                   Car_SetSlaveDisplacement(left_dev, half_comp);
               }
            }
            
            /* 前进完成，等待稳定 */
            osDelay(2000);
        }

        /* 左转:使用角度判断,每2ms检查一次 */
        {
            /* 旋转开始前，使用当前基准的角度作为参考，旋转目标为 current_base - g_target_angle */
            float prev_angle = TIM_GetAngle();
            float accumulated_angle = 0.0f;

            /* 发送左转命令(开始旋转) */
            Car_TurnLeft(g_turn_distance);

            /* 每2ms检查一次角度变化并累加绝对值,直到达到目标角度 */
            while (1)
            {
                osDelay(2);

                float current_angle = TIM_GetAngle();
                float delta = current_angle - prev_angle;

                if (delta > 180.0f) {
                    delta -= 360.0f;
                } else if (delta < -180.0f) {
                    delta += 360.0f;
                }

                accumulated_angle += fabsf(delta);
                prev_angle = current_angle;

                if (accumulated_angle >= g_target_angle)
                {
                    Car_Stop();
                    break;
                }
            }
	        osDelay(1000);
            /* 旋转完成后的角度校准 - 使用PID补偿旋转误差 */
            {
                uint8_t left_dev = Car_GetLeftDevID();
                uint8_t right_dev = Car_GetRightDevID();
                    /* 计算目标角度：使用直走时读取的 base_angle 减去旋转角度（左转），即 base_angle - 180° */
                    float target_angle = base_angle - g_target_angle;  /* 左转为负 */
                
                /* 归一化目标角度到 ±180度范围 */
                while (target_angle > 180.0f) {
                    target_angle -= 360.0f;
                }
                while (target_angle < -180.0f) {
                    target_angle += 360.0f;
                }
                
                /* 初始化PID控制器 */
                PID_State_t pid_state;
                PID_Reset(&pid_state);
                
                /* 循环校准3秒 */
                uint32_t elapsed = 0;
                const uint32_t calibration_time = 2000;  /* 3秒校准时间 */
                const float dt = 0.05f;  /* 50ms = 0.05秒 */
                int slave_cleared = 0;
                
                while (elapsed < calibration_time) {
                    osDelay(50);
                    elapsed += 50;
                    
                    /* 读取当前角度 */
                    float current_angle = TIM_GetAngle();
                    
                    /* 计算角度误差（目标角度 - 实际角度） */
                    float angle_error = target_angle - current_angle;
                    
                    /* 处理角度跨越±180度的情况 */
                    if (angle_error > 180.0f) {
                        angle_error -= 360.0f;
                    } else if (angle_error < -180.0f) {
                        angle_error += 360.0f;
                    }
                    
                    /* 导出到全局，便于调试查看 */
                    g_angle_error = angle_error;
                    
                    /* PID计算补偿量 */
                    float compensation = PID_Calculate(&pid_state, angle_error, dt);
                    
                    /* 使用从位移进行旋转角度校正 */
                    if (fabsf(compensation) > 0.1f) {
                        /* 补偿量分配给两个轮子 */
                        float half_comp = compensation / 1.0f;
                        /* 仅调整左轮进行角度校正 */
                        Car_SetSlaveDisplacement(left_dev, half_comp);
                    }
                }
                /* 无需切换基准：每次直走阶段会重新读取基准 */
            }
        }

        /* 小间隔,确保角度稳定 */
        osDelay(500);
    }
}

