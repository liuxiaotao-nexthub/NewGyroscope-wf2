#include "tim.h"
#include "xv7001bb.h"
#include <math.h>

/* 使用 TIM2 每 2ms 中断 */
TIM_HandleTypeDef htim2;

/* 积分状态结构体 */
IntegratorState integrator = {0.0f, 0.0f};

/* 环形缓冲区与滑动窗口统计量 */
float zerobuf[ZEROBUF_SIZE];
int zerobuf_idx = 0;
int zerobuf_count = 0;
float zerobuf_sum = 0.0f;
float bias = 0.0f;

int bias_initialized = 0;

/* 调试用全局变量 */
float gyro_mean = 0.0f;
/* 仅保留用于调试的均值 */
// float gyro_var; // variance removed

/* 保留分段统计变量以匹配头文件和使用处（虽然不使用方差） */
float segment_means[STABLE_CHECK_TIMES] = {0.0f};
int segment_index = 0;
int samples_in_segment = 0;
float segment_sum = 0.0f;

/* 消息队列句柄 */
osMessageQId gyroQueueHandle;

void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2000 - 1;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        while (1);
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* 创建消息队列（队列长度10，每个元素是float） */
    osMessageQDef(gyroQueue, 10, float);
    gyroQueueHandle = osMessageCreate(osMessageQ(gyroQueue), NULL);

    HAL_TIM_Base_Start_IT(&htim2);
}

/* TIM2中断回调 - 只读取角速度并发送到队列 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 读取原始角速度 (°/s) */
        float raw_rate = XV7001BB_ReadAngularRate();

        /* 发送到消息队列，如果队列满了则覆盖（不等待） */
        osMessagePut(gyroQueueHandle, *(uint32_t*)&raw_rate, 0);
    }
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

float TIM_GetAngle(void)
{
    return integrator.angle;
}
