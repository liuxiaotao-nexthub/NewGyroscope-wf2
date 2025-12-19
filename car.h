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

/* 小车任务线程句柄（在 main 中创建） */
extern osThreadId CarLockTaskHandle;

/* 电机位移解析函数：从 ID 0x409 的 8 字节数据中解析位移值
 * 参数：data - 8字节CAN数据
 *       dev_id - 输出参数，设备号（0x01或0x02）
 * 返回：位移值（单位：mm，浮点数）
 */
float Car_ParseMotorPosition(const uint8_t *data, uint8_t *dev_id);

/* 发送读取电机位移命令（ID 0x408）
 * 参数：dev_id - 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
 */
void Car_ReadMotorPosition(uint8_t dev_id);

/* 交换左右轮设备号（用于校准后纠正） */
void Car_SwapDeviceID(void);

/* 获取当前左轮设备号 */
uint8_t Car_GetLeftDevID(void);

/* 获取当前右轮设备号 */
uint8_t Car_GetRightDevID(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */
