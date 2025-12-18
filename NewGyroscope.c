/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include <math.h>
#include <string.h>
#include "spi.h"
#include "can.h"
#include "xv7001bb.h"
#include "can_task.h"
#include "tim.h"
#include "car.h"

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
    CAR_STATE_ENABLE_POWER,     /* 上电使能状态：发送电源使能命令（ID 0x413）并发送锁定命令（ID 0x60D） */
    CAR_STATE_DONE              /* 完成状态：任务自挂起 */
} CarState_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId MainTaskHandle;
osThreadId GyroProcessTaskHandle;

/* 小车 CAN 消息队列句柄 */
osMessageQId CarCanQueueHandle = NULL;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Main_Task(void const *argument);
static void Gyro_ProcessTask(void const *argument);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2 (APB1 = 36MHz)
  *            APB2 Prescaler                 = 1 (APB2 = 72MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLL Multiplier                 = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		while (1);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		while (1);
	}
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32F1xx HAL library initialization */
	HAL_Init();
	
	/* Configure the system clock to 72 MHz */
	SystemClock_Config();
	
	/* Enable GPIOB Clock */
	__GPIOB_CLK_ENABLE();
	
	/* Configure PB9 as output */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialize SPI */
	SPI_Init();

	/* Initialize CAN */
	CAN_Init();

	/* Initialize XV7001BB */
	XV7001BB_Init();

	/* Initialize TIM2 for 2ms integration interrupts */
	TIM2_Init();

	/* 创建小车 CAN 消息队列（队列深度 4） */
	osMessageQDef(carCanQueue, 4, CarCanMsg_t);
	CarCanQueueHandle = osMessageCreate(osMessageQ(carCanQueue), NULL);

	/* Create CAN send task */
	osThreadDef(CANSEND, CAN_SendTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate(osThread(CANSEND), NULL);

	/* Create gyroscope processing task */
	osThreadDef(GYROPROCESS, Gyro_ProcessTask, osPriorityHigh, 0, 512);
	GyroProcessTaskHandle = osThreadCreate(osThread(GYROPROCESS), NULL);

	/* Create car lock task */
	osThreadDef(CARLOCK, Car_LockTask, osPriorityNormal, 0, 256);
	CarLockTaskHandle = osThreadCreate(osThread(CARLOCK), NULL);

	/* Main task definition */
	osThreadDef(MAIN, Main_Task, osPriorityNormal, 0, 64);
	MainTaskHandle = osThreadCreate(osThread(MAIN), NULL);
	
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	for (;;)
		;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

/**
  * @brief  主任务 - 翻转PB9的LED
  * @param  argument 未使用
  * @retval 无
  */
static void Main_Task(void const *argument)
{
    (void) argument;
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
        osDelay(100);
    }
}

/**
  * @brief  陀螺仪数据处理任务 - 使用环形缓冲区滑动窗口检测零漂
  * @param  argument: 任务参数（未使用）
  * @retval 无
  */
static void Gyro_ProcessTask(void const *argument)
{
    (void)argument;
    osEvent evt;
    float raw_rate;
    
    for (;;)
    {
        /* 从消息队列接收角速度数据，永久等待 */
        evt = osMessageGet(gyroQueueHandle, osWaitForever);
        
        if (evt.status == osEventMessage)
        {
            /* 提取角速度值 */
            raw_rate = *(float*)&evt.value.v;

            /* === 环形缓冲区滑动窗口更新 === */
            if (zerobuf_count < ZEROBUF_SIZE)
            {
                /* 初始填充阶段 */
                zerobuf[zerobuf_idx] = raw_rate;
                zerobuf_sum += raw_rate;
                zerobuf_sumsq += raw_rate * raw_rate;
                zerobuf_count++;
                zerobuf_idx++;
                if (zerobuf_idx >= ZEROBUF_SIZE) zerobuf_idx = 0;
            }
            else
            {
                /* 滑动窗口更新：移除最旧数据，加入新数据 */
                float old_value = zerobuf[zerobuf_idx];
                zerobuf_sum -= old_value;
                zerobuf_sumsq -= old_value * old_value;
                
                zerobuf[zerobuf_idx] = raw_rate;
                zerobuf_sum += raw_rate;
                zerobuf_sumsq += raw_rate * raw_rate;
                
                zerobuf_idx++;
                if (zerobuf_idx >= ZEROBUF_SIZE) zerobuf_idx = 0;
                
                /* 计算当前滑动窗口的均值与方差 */
                gyro_mean = zerobuf_sum / (float)ZEROBUF_SIZE;
                float msq = zerobuf_sumsq / (float)ZEROBUF_SIZE;
                gyro_var = msq - gyro_mean * gyro_mean;
                if (gyro_var < 0.0f) gyro_var = 0.0f;
                
                /* === 静止检测状态机 === */
                if (gyro_var < VAR_THRESHOLD && fabsf(gyro_mean) < MEAN_THRESHOLD)
                {
                    /* 满足静止条件，开始分段统计 */
                    
                    /* 累加当前样本到当前段 */
                    segment_sum += raw_rate;
                    samples_in_segment++;
                    
                    /* 当前段收集满200个样本 */
                    if (samples_in_segment >= SEGMENT_SIZE)
                    {
                        /* 计算当前段的平均值 */
                        segment_means[segment_index] = segment_sum / (float)SEGMENT_SIZE;
                        
                        /* 移动到下一段 */
                        segment_index++;
                        segment_sum = 0.0f;
                        samples_in_segment = 0;
                        
                        /* 如果已经收集完5段数据（共1000个样本） */
                        if (segment_index >= STABLE_CHECK_TIMES)
                        {
                            /* 取中间段（第3段，索引2）的平均值更新 bias */
                            float middle_segment_mean = segment_means[2];
                        
                            /* 使用 EMA 滤波平滑更新 bias */
                            if (!bias_initialized)
                            {
                                /* 首次初始化，直接使用测量值 */
                                bias = middle_segment_mean;
                                bias_initialized = 1;
                            }
                            else
                            {
                                /* 已初始化，使用 EMA 滤波 */
                                bias = BIAS_EMA_ALPHA * middle_segment_mean + (1.0f - BIAS_EMA_ALPHA) * bias;
                            }
                            
                            /* 重置分段统计，准备下一轮检测 */
                            segment_index = 0;
                            segment_sum = 0.0f;
                            samples_in_segment = 0;
                        }
                    }
                }
                else
                {
                    /* 不满足静止条件，重置所有分段统计 */
                    segment_index = 0;
                    segment_sum = 0.0f;
                    samples_in_segment = 0;
                }
            }

            /* --- 去零漂后进行梯形积分 --- */
            float corrected = raw_rate - bias;
            float dt = 0.002f;
            integrator.angle += (integrator.prev_rate + corrected) * 0.5f * dt;
            integrator.prev_rate = corrected;
	        
	        /* 角度归一化到 [-180°, 180°] 防止溢出 */
	        integrator.angle = fmodf(integrator.angle + 180.0f, 360.0f) - 180.0f;
	        if (integrator.angle < -180.0f) {
		        integrator.angle += 360.0f;
	        }
        }
    }
}

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
                            /* 发送左轮绑定命令 */
                            uint8_t buf[8];
                            memcpy(buf, sn_left, 7);
                            buf[7] = CAR_DEV_LEFT;
                            CAN_SendData(0x313, buf, 8);
                        }
                        else if (!have_right)
                        {
                            memcpy(sn_right, pMsg->data, 7);
                            have_right = 1;
                            /* 发送右轮绑定命令 */
                            uint8_t buf[8];
                            memcpy(buf, sn_right, 7);
                            buf[7] = CAR_DEV_RIGHT;
                            CAN_SendData(0x313, buf, 8);
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
                   B4 BF CF AA 00 5F DEV DEV（共 8 字节），分别初始化左、右电机。*/
                {
                    uint8_t buf_left[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, CAR_DEV_LEFT, CAR_DEV_LEFT};
                    CAN_SendData(0x316, buf_left, 8);
                    osDelay(1);

                    uint8_t buf_right[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, CAR_DEV_RIGHT, CAR_DEV_RIGHT};
                    CAN_SendData(0x316, buf_right, 8);
                    osDelay(1);

                    car_state = CAR_STATE_ENABLE_POWER; /* 跳转到上电使能状态 */
                }
                break;

            case CAR_STATE_ENABLE_POWER:
                /* 状态说明：对左右电机发送上电使能命令（ID 0x413），数据为
                   5A 00 00 00 DEV（使能），发送后再发送锁定命令（ID 0x60D，07 00）。
                */
                {
                    uint8_t ena_left[5] = {0x5A, 0x00, 0x00, 0x00, CAR_DEV_LEFT};
                    CAN_SendData(0x413, ena_left, 5);
                    osDelay(1);

                    uint8_t ena_right[5] = {0x5A, 0x00, 0x00, 0x00, CAR_DEV_RIGHT};
                    CAN_SendData(0x413, ena_right, 5);
                    osDelay(1);

                    uint8_t lock[2] = {0x07, 0x00};
                    CAN_SendData(0x60D, lock, 2);

                    car_state = CAR_STATE_DONE; /* 完成后转到 DONE 状态 */
                }
                break;

            case CAR_STATE_DONE:
                /* 状态说明：完成绑定、初始化与上电使能并发送锁定命令后，任务自挂起 */
                vTaskSuspend(NULL);
                break;

            default:
                break;
        }

        osDelay(10);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
