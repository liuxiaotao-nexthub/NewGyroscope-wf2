#include "car.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

/* 该文件作为小车底层驱动接口，任务实现已移至 NewGyroscope.c，后续在此添加硬件相关实现 */

/* 小车任务句柄（在 main 中创建） */
osThreadId CarLockTaskHandle = NULL;

/* 当前左右轮设备号（可能在校准后被交换） */
static uint8_t current_left_dev = CAR_DEV_LEFT;
static uint8_t current_right_dev = CAR_DEV_RIGHT;

/* 发送小车锁定命令（ID 0x60D，数据: 07 00） */
static void send_lock_cmd(void)
{
    uint8_t data[2] = {0x07, 0x00};
    CAN_SendData(0x60D, data, 2);
}

/* 绑定 SN：发送 ID 0x313，数据: 7 字节 SN + 1 字节设备号 */
static void send_bind_sn(const uint8_t *sn, uint8_t dev)
{
    uint8_t buf[8];
    memcpy(buf, sn, 7);
    buf[7] = dev;
    CAN_SendData(0x313, buf, 8);
}

/* 初始化电机参数：发送 ID 0x316，数据 B4 BF CF AA 00 5F DEV DEV（8 字节） */
static void send_init_motor(uint8_t dev)
{
    uint8_t buf[8] = {0xB4, 0xBF, 0xCF, 0xAA, 0x00, 0x5F, dev, dev};
    CAN_SendData(0x316, buf, 8);
}

/* 电源使能：发送 ID 0x413，数据: 5A/4A + 00 00 00 + DEV（5 字节） */
static void send_enable_motor(uint8_t enable, uint8_t dev)
{
    uint8_t buf[5];
    buf[0] = enable; /* 0x5A 使能，0x4A 失能 */
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x00;
    buf[4] = dev;
    CAN_SendData(0x413, buf, 5);
}

/**
  * @brief  解析电机位移数据（ID 0x409）
  * @param  data: 8字节CAN数据
  *         data[0]: 设备号
  *         data[2-5]: 位移值（小端，补码，单位0.1mm）
  * @param  dev_id: 输出设备号
  * @retval 位移值（单位：mm）
  */
float Car_ParseMotorPosition(const uint8_t *data, uint8_t *dev_id)
{
    /* 提取设备号 */
    if (dev_id != NULL)
    {
        *dev_id = data[0];
    }
    
    /* 提取位移值（小端格式，4字节补码） */
    int32_t pos_raw = (int32_t)(data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24));
    
    /* 转换为 mm（原始单位是 0.1mm） */
    float position_mm = (float)pos_raw / 10.0f;
    
    return position_mm;
}

/**
  * @brief  发送读取电机位移命令（ID 0x408）
  * @param  dev_id: 设备号（CAR_DEV_LEFT 或 CAR_DEV_RIGHT）
  * @retval 无
  */
void Car_ReadMotorPosition(uint8_t dev_id)
{
    uint8_t cmd[2];
    cmd[0] = dev_id;
    cmd[1] = 0x08;
    CAN_SendData(0x408, cmd, 2);
}

/**
  * @brief  交换左右轮设备号（校准后纠正）
  * @param  无
  * @retval 无
  */
void Car_SwapDeviceID(void)
{
    uint8_t temp = current_left_dev;
    current_left_dev = current_right_dev;
    current_right_dev = temp;
}

/**
  * @brief  获取当前左轮设备号
  * @param  无
  * @retval 左轮设备号
  */
uint8_t Car_GetLeftDevID(void)
{
    return current_left_dev;
}

/**
  * @brief  获取当前右轮设备号
  * @param  无
  * @retval 右轮设备号
  */
uint8_t Car_GetRightDevID(void)
{
    return current_right_dev;
}

/* 任务实现已移至 NewGyroscope.c */
