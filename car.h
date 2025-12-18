#ifndef __CAR_H
#define __CAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <../CMSIS_RTOS/cmsis_os.h>

/* AGV 设备号 */
#define CAR_DEV_LEFT  0x01
#define CAR_DEV_RIGHT 0x02

/* 小车锁定任务入口（状态机） */
void Car_LockTask(void const *argument);

/* 小车任务线程句柄（在 main 中创建并挂起，接收到 0x312 时唤醒） */
extern osThreadId CarLockTaskHandle;

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */
