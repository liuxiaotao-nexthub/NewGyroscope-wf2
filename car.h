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
extern osThreadId CarTestTaskHandle;

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

/* ========== 电机运动控制接口 ========== */

/**
  * @brief  小车前进
  * @param  distance: 前进距离（单位：mm，正数）
  * @retval 无
  * @note   前进：左轮负数位移，右轮正数位移
  */
void Car_MoveForward(float distance);

/**
  * @brief  小车后退
  * @param  distance: 后退距离（单位：mm，正数）
  * @retval 无
  * @note   后退：左轮正数位移，右轮负数位移
  */
void Car_MoveBackward(float distance);

/**
  * @brief  小车左转
  * @param  distance: 转动距离（单位：mm，正数）
  * @retval 无
  * @note   左转：两个轮子都是正数位移
  */
void Car_TurnLeft(float distance);

/**
  * @brief  小车右转
  * @param  distance: 转动距离（单位：mm，正数）
  * @retval 无
  * @note   右转：两个轮子都是负数位移
  */
void Car_TurnRight(float distance);

/* ========== 电机参数设置接口 ========== */

/**
  * @brief  设置轮径
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  diameter: 轮径（单位：0.01mm）
  * @retval 无
  * @note   例如：25.75mm 应传入 2575
  */
void Car_SetWheelDiameter(uint8_t dev_id, uint16_t diameter);

/**
  * @brief  设置加速度
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  acceleration: 加速度（单位：0.1mm/s?）
  * @retval 无
  * @note   例如：2000mm/s? 应传入 20000
  */
void Car_SetAcceleration(uint8_t dev_id, uint32_t acceleration);

/**
  * @brief  设置速度
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  velocity: 速度（单位：0.1mm/s）
  * @retval 无
  * @note   例如：1000mm/s 应传入 10000
  */
void Car_SetVelocity(uint8_t dev_id, uint32_t velocity);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */
