#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* 零漂检测参数（滑动窗口大小及滑动窗口周期） */
#define ZEROBUF_SIZE 300               /* 滑动窗口大小：1000样本 × 2ms = 2秒 */
#define STABLE_CHECK_TIMES 5            /* 检5次，每次200个样本 */
#define SEGMENT_SIZE 50                /* 每次段采样数量 */

/* 稳定性阈值 */
#define VAR_THRESHOLD 5.0f
#define MEAN_THRESHOLD 0.1f


/* bias EMA 滤波参数 */
#define BIAS_EMA_ALPHA 1.0f             /* EMA 系数: 0.1 = 10% 新值 + 90% 旧值 */

/* 积分状态结构体 */
typedef struct {
    float prev_rate;
    float angle;
} IntegratorState;

/* 全局变量声明 */
extern IntegratorState integrator;
extern float zerobuf[ZEROBUF_SIZE];     /* 零漂缓冲区 */
extern int zerobuf_idx;                 /* 零漂缓冲区当前索引 */
extern int zerobuf_count;               /* 缓冲区有效样本数 */
extern float zerobuf_sum;               /* 缓冲区总和 */
extern float zerobuf_sumsq;             /* 缓冲区平方和 */
extern float bias;
extern int bias_initialized;

/* 调试用全局变量 */
extern float gyro_mean;                 /* 当前滑动窗口均值 */
extern float gyro_var;                  /* 当前滑动窗口方差 */
extern float segment_means[STABLE_CHECK_TIMES];  /* 记录5次的均值 */
extern int segment_index;               /* 当前分段的段索引 */
extern int samples_in_segment;          /* 当前分段已处理样本数 */
extern float segment_sum;               /* 当前段的累加和 */


/* 消息队列句柄 */
extern osMessageQId gyroQueueHandle;

void TIM2_Init(void);
float TIM_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif
