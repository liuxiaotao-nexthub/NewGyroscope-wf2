/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机控制模块实现
  ******************************************************************************
  */

#include "motor.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

/* ========== 公共函数实现 ========== */

/**
  * @brief  请求电机SN码（ID 0x60D，数据: 07 00）
  * @retval None
  */
void Motor_RequestSN(void)
{
    uint8_t data[2] = {0x07, 0x00};
    CAN_SendData(0x60D, data, 2);
}

/**
  * @brief  绑定电机SN（ID 0x313，数据: 7字节SN + 1字节设备号）
  * @param  sn: 7字节SN号
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_BindSN(const uint8_t *sn, uint8_t dev_id)
{
    uint8_t buf[8];
    memcpy(buf, sn, 7);
    buf[7] = dev_id;
    CAN_SendData(0x313, buf, 8);
}

/**
  * @brief  初始化电机参数（ID 0x316，数据: B4 BF CF AA 00 5F DEV DEV）
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_InitParams(uint8_t dev_id)
{
    uint8_t buf[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, dev_id, dev_id};

    /* 连续发送3遍，增加短延时以保证总线稳定 */
    for (int i = 0; i < 3; i++) {
        CAN_SendData(0x316, buf, 8);
        osDelay(1);
    }
}

/**
  * @brief  电机使能/失能（ID 0x413，数据: 5A/4A + 00 00 00 + DEV）
  * @param  enable: MOTOR_ENABLE(0x5A) 或 MOTOR_DISABLE(0x4A)
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_Enable(uint8_t enable, uint8_t dev_id)
{
    uint8_t buf[5];
    buf[0] = enable;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = dev_id;

    /* 连续发送3遍，增加短延时以保证总线稳定 */
    for (int i = 0; i < 3; i++) {
        CAN_SendData(0x413, buf, 5);
        osDelay(1);
    }
}

/**
  * @brief  解析电机位移数据（ID 0x409 响应）
  * @param  data: 8字节CAN数据
  *         data[0]: 设备号
  *         data[2-5]: 位移值（小端，补码，单位0.1mm）
  * @param  dev_id: 输出设备号（可为NULL）
  * @retval 位移值（单位：mm）
  */
float Motor_ParsePosition(const uint8_t *data, uint8_t *dev_id)
{
    if (dev_id != NULL)
    {
        *dev_id = data[0];
    }
    
    /* 提取位移值（小端格式，4字节补码） */
    int32_t pos_raw = (int32_t)(data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24));
    
    /* 转换为 mm（原始单位是 0.1mm） */
    return (float)pos_raw / 10.0f;
}

/**
  * @brief  请求电机返回位移值（ID 0x408，电机通过 0x409 响应）
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_RequestPosition(uint8_t dev_id)
{
    uint8_t cmd[2];
    cmd[0] = dev_id;
    cmd[1] = 0x08;
    CAN_SendData(0x408, cmd, 2);
}

/**
  * @brief  发送位移命令（ID 0x416）
  * @param  dev_id: 设备号
  * @param  displacement: 位移值（单位：mm，正负表示方向）
  * @retval None
  */
void Motor_SendDisplacement(uint8_t dev_id, float displacement)
{
    /* 将 mm 转换为 0.1mm 单位的整数（补码） */
    int32_t displacement_raw = (int32_t)(displacement * 10.0f);
    
    uint8_t cmd[6];
    cmd[0] = (uint8_t)(displacement_raw & 0xFF);
    cmd[1] = (uint8_t)((displacement_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((displacement_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((displacement_raw >> 24) & 0xFF);
    cmd[4] = dev_id;
    cmd[5] = 0x00;
    
    CAN_SendData(0x416, cmd, 6);
}

/**
  * @brief  停止电机（发送位移0.1mm）
  * @param  dev_id: 设备号
  * @retval None
  */
void Motor_Stop(uint8_t dev_id)
{
    for (int i = 0; i < 2; i++) {
        Motor_SendDisplacement(dev_id, 0.1f);
        osDelay(1);
    }
}

/**
  * @brief  设置轮径（ID 0x408）
  * @param  dev_id: 设备号
  * @param  diameter: 轮径（单位：mm，发送时转换为0.01mm）
  * @retval None
  */
void Motor_SetWheelDiameter(uint8_t dev_id, uint16_t diameter)
{
    /* 将mm转换为0.01mm单位 */
    uint16_t diameter_raw = diameter * 100;
    
    uint8_t cmd[4];
    cmd[0] = dev_id;
    cmd[1] = 0x24;
    cmd[2] = (uint8_t)(diameter_raw & 0xFF);
    cmd[3] = (uint8_t)((diameter_raw >> 8) & 0xFF);
    
    for (int i = 0; i < 3; i++)
    {
        CAN_SendData(0x408, cmd, 4);
        osDelay(1);
    }
}

/**
  * @brief  设置加速度（ID 0x414）
  * @param  dev_id: 设备号
  * @param  acceleration: 加速度（单位：mm/s²，发送时转换为0.1mm/s²）
  * @retval None
  */
void Motor_SetAcceleration(uint8_t dev_id, uint32_t acceleration)
{
    /* 将mm/s²转换为0.1mm/s²单位 */
    uint32_t acc_raw = acceleration * 10;
    
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(acc_raw & 0xFF);
    cmd[1] = (uint8_t)((acc_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((acc_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((acc_raw >> 24) & 0xFF);
    cmd[4] = dev_id;
    
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x414, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置速度（ID 0x415）
  * @param  dev_id: 设备号
  * @param  velocity: 速度（单位：mm/s，发送时转换为0.1mm/s）
  * @retval None
  */
void Motor_SetVelocity(uint8_t dev_id, uint32_t velocity)
{
    /* 将mm/s转换为0.1mm/s单位 */
    uint32_t vel_raw = velocity * 10;
    
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(vel_raw & 0xFF);
    cmd[1] = (uint8_t)((vel_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((vel_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((vel_raw >> 24) & 0xFF);
    cmd[4] = dev_id;
    
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x415, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置从位移（ID 0x419）
  * @param  dev_id: 设备号
  * @param  displacement: 从位移（单位：mm）
  * @retval None
  */
void Motor_SetSlaveDisplacement(uint8_t dev_id, float displacement)
{
    int32_t displacement_raw = (int32_t)(displacement * 10.0f);
    
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(displacement_raw & 0xFF);
    cmd[1] = (uint8_t)((displacement_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((displacement_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((displacement_raw >> 24) & 0xFF);
    cmd[4] = dev_id;
    
    CAN_SendData(0x419, cmd, 5);
}

/**
  * @brief  设置从速度（ID 0x418）
  * @param  dev_id: 设备号
  * @param  velocity: 从速度（单位：0.1mm/s）
  * @retval None
  */
void Motor_SetSlaveVelocity(uint8_t dev_id, uint32_t velocity)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(velocity & 0xFF);
    cmd[1] = (uint8_t)((velocity >> 8) & 0xFF);
    cmd[2] = (uint8_t)((velocity >> 16) & 0xFF);
    cmd[3] = (uint8_t)((velocity >> 24) & 0xFF);
    cmd[4] = dev_id;
    
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x418, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置从加速度（ID 0x417）
  * @param  dev_id: 设备号
  * @param  acceleration: 从加速度（单位：0.1mm/s²）
  * @retval None
  */
void Motor_SetSlaveAcceleration(uint8_t dev_id, uint32_t acceleration)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(acceleration & 0xFF);
    cmd[1] = (uint8_t)((acceleration >> 8) & 0xFF);
    cmd[2] = (uint8_t)((acceleration >> 16) & 0xFF);
    cmd[3] = (uint8_t)((acceleration >> 24) & 0xFF);
    cmd[4] = dev_id;
    
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x417, cmd, 5);
        osDelay(1);
    }
}
