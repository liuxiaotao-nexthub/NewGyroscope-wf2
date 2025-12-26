#ifndef __CAN_TASK_H
#define __CAN_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <../CMSIS_RTOS/cmsis_os.h>
#include <stdint.h>

/* 全局缓存，可从陀螺仪任务更新、陀螺仪发送任务读取（无互斥保护） */
extern volatile int32_t can_rate_fp;
extern volatile int32_t can_angle_fp;
extern volatile int32_t can_temp_fp; /* 温度定点数 *100 */
extern volatile uint8_t can_statusReg;

void CAN_ControlTask(void const *argument);
void CAN_SendTask(void const *argument);

#ifdef __cplusplus
}
#endif

#endif
