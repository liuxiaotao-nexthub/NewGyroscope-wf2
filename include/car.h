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

/* 解析位移数据：解析电机 ID 0x409 的 8 字节数据，获得位移值
 * 参数：data - 8字节CAN数据
 *       dev_id - 输出参数：设备号（0x01或0x02）
 * 返回：位移值（单位：mm）（负数反向）
 */
float Car_ParseMotorPosition(const uint8_t *data, uint8_t *dev_id);

/* 发送读取电机位移（ID 0x408）
 * 参数：dev_id - 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
 */
void Car_ReadMotorPosition(uint8_t dev_id);

/* 交换左右轮设备号（用于校准后修正） */
void Car_SwapDeviceID(void);

/* 获取当前左轮设备号 */
uint8_t Car_GetLeftDevID(void);

/* 获取当前右轮设备号 */
uint8_t Car_GetRightDevID(void);

/* ========== 小车运动控制接口 ========== */

/**
  * @brief  小车前进
  * @param  distance: 前进距离（单位：mm）（绝对值）
  * @retval 无
  * @note   前进：两轮都是正向位移
  */
void Car_MoveForward(float distance);

/**
  * @brief  小车后退
  * @param  distance: 后退距离（单位：mm）（绝对值）
  * @retval 无
  * @note   后退：两轮都是负向位移，传入正向位移
  */
void Car_MoveBackward(float distance);

/**
  * @brief  小车左转
  * @param  distance: 转弯距离（单位：mm）（绝对值）
  * @retval 无
  * @note   左转：左轮反向，右轮正向位移
  */
void Car_TurnLeft(float distance);

/**
  * @brief  小车右转
  * @param  distance: 转弯距离（单位：mm）（绝对值）
  * @retval 无
  * @note   右转：右轮反向，左轮正向位移
  */
void Car_TurnRight(float distance);

/* ========== 电机参数设置接口 ========== */

/**
  * @brief  设置轮径
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  diameter: 轮径（单位：0.01mm）
  * @retval 无
  */
void Car_SetWheelDiameter(uint8_t dev_id, uint16_t diameter);

/**
  * @brief  设置轮距
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  wheelbase: 轮距（单位：0.01mm）
  * @retval 无
  */
void Car_SetWheelbase(uint8_t dev_id, uint32_t wheelbase);

/**
  * @brief  设置速度
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  velocity: 速度（单位：0.1mm/s）
  * @retval 无
  */
void Car_SetVelocity(uint8_t dev_id, uint32_t velocity);

/**
  * @brief  设置加速度
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  acceleration: 加速度（单位：0.1mm/s²）
  * @retval 无
  */
void Car_SetAcceleration(uint8_t dev_id, uint32_t acceleration);

/**
  * @brief  设置电机运动到指定位置
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @param  position: 目标位置（单位：mm，正数前进，负数后退）
  * @retval 无
  */
void Car_SetMotorPosition(uint8_t dev_id, float position);

/**
  * @brief  设置从位移
  * @param  dev_id: 设备号
  * @param  displacement: 从位移（单位：mm，正数或负数）
  * @retval 无
  */
void Car_SetSlaveDisplacement(uint8_t dev_id, float displacement);

/**
  * @brief  设置从速度
  * @param  dev_id: 设备号
  * @param  velocity: 从速度（单位：0.1mm/s）
  * @retval 无
  */
void Car_SetSlaveVelocity(uint8_t dev_id, uint32_t velocity);

/**
  * @brief  设置从加速度
  * @param  dev_id: 设备号
  * @param  acceleration: 从加速度（单位：0.1mm/s²）
  * @retval 无
  */
void Car_SetSlaveAcceleration(uint8_t dev_id, uint32_t acceleration);

/**
  * @brief  小车停止
  * @param  无
  * @retval 无
  */
void Car_Stop(void);

/**
  * @brief  等待电机运动完成
  * @param  timeout_ms: 超时时间（毫秒）
  * @retval 0: 成功，-1: 超时
  */
int Car_WaitMotionComplete(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */

/************************ 文件结束 ****/
