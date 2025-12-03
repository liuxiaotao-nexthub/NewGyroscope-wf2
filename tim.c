#include "tim.h"
#include "xv7001bb.h"
#include <math.h>

/* 使用 TIM2 每 2ms 中断 */
TIM_HandleTypeDef htim2;

/* 积分状态结构体 */
typedef struct {
    float prev_rate; /* 上次角速度 (°/s) */
    float angle;     /* 积分角度 (°) */
} IntegratorState;

static IntegratorState integrator = {0.0f, 0.0f};

/* 零漂估计参数（缓冲与统计量） */
#define ZEROBUF_SIZE 200
static float zerobuf[ZEROBUF_SIZE];
static int zerobuf_idx = 0;
static int zerobuf_count = 0;
static float zerobuf_sum = 0.0f;
static float zerobuf_sumsq = 0.0f;
static float bias = 0.0f; /* 当前零漂估计 (°/s) */

/* 静止检测参数（放宽条件以容忍振动） */
#define VAR_THRESHOLD 2.0f      /* 方差阈值，放宽以容忍振动 */
#define MEAN_THRESHOLD 1.0f     /* 均值阈值，放宽 */
#define STABLE_COUNT_MIN 250    /* 需连续 0.5s 静止 */

/* 慢速自适应 bias 更新（抑制振动影响） */
#define BIAS_UPDATE_ALPHA 0.01f /* 慢速低通系数 */

static int stable_count = 0;  /* 连续静止计数器 */

/* 一阶低通滤波器（用于平滑角速度） */
typedef struct {
    float y;
    float alpha;
} LPF_1st;

static LPF_1st rate_lpf = {0.0f, 0.0f};

void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1; // 72MHz/72 = 1MHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 2000 - 1;  // 1MHz / 2000 = 500Hz -> 2ms
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        while (1);
    }

    HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* 初始化软件低通滤波器：15Hz 截止频率 */
    float fc = 15.0f;  /* 截止频率，抑制振动 */
    float dt = 0.002f;
    float RC = 1.0f / (2.0f * 3.14159265f * fc);
    rate_lpf.alpha = dt / (RC + dt);
    rate_lpf.y = 0.0f;

    HAL_TIM_Base_Start_IT(&htim2);
}

/* TIM2中断回调 - 添加软件低通 + 慢速自适应 bias */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 读取原始角速度 (°/s) */
        float raw_rate = XV7001BB_ReadAngularRate();

        /* === 软件低通滤波（抑制高频振动） === */
        rate_lpf.y += rate_lpf.alpha * (raw_rate - rate_lpf.y);
        float filtered_rate = rate_lpf.y;

        /* --- 将滤波后样本加入循环缓冲 --- */
        if (zerobuf_count < ZEROBUF_SIZE)
        {
            zerobuf[zerobuf_idx] = filtered_rate;
            zerobuf_sum += filtered_rate;
            zerobuf_sumsq += filtered_rate * filtered_rate;
            zerobuf_count++;
        }
        else
        {
            float old = zerobuf[zerobuf_idx];
            zerobuf_sum -= old;
            zerobuf_sumsq -= old * old;
            zerobuf[zerobuf_idx] = filtered_rate;
            zerobuf_sum += filtered_rate;
            zerobuf_sumsq += filtered_rate * filtered_rate;
        }

        zerobuf_idx++;
        if (zerobuf_idx >= ZEROBUF_SIZE) zerobuf_idx = 0;

        /* --- 计算均值与方差 --- */
        float mean = 0.0f;
        float var = 0.0f;
        if (zerobuf_count >= ZEROBUF_SIZE)
        {
            mean = zerobuf_sum / (float)ZEROBUF_SIZE;
            float msq = zerobuf_sumsq / (float)ZEROBUF_SIZE;
            var = msq - mean * mean;
            if (var < 0.0f) var = 0.0f;
        }

        /* === 改进的静止检测 === */
        if (zerobuf_count >= ZEROBUF_SIZE && 
            var < VAR_THRESHOLD && 
            fabsf(mean) < MEAN_THRESHOLD)
        {
            stable_count++;
            
            /* 连续静止足够久后，使用慢速自适应更新 bias */
            if (stable_count >= STABLE_COUNT_MIN)
            {
                /* 慢速低通更新，而非直接替换（抑制振动尖峰影响） */
                bias += BIAS_UPDATE_ALPHA * (mean - bias);
            }
        }
        else
        {
            stable_count = 0;
        }

        /* --- 去零漂后进行梯形积分 --- */
        float corrected = filtered_rate - bias;
        float dt = 0.002f;
        integrator.angle += (integrator.prev_rate + corrected) * 0.5f * dt;
        integrator.prev_rate = corrected;
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
