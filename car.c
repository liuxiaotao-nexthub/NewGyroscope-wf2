#include "car.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

/* 该文件作为小车底层驱动接口，任务实现已移至 NewGyroscope.c，后续在此添加硬件相关实现 */


extern float g_linear_velocity;     /* 直线运动速度 (mm/s) */
extern float g_rotation_velocity;   /* 旋转速度 (mm/s) */
extern float g_acceleration;        /* 加速度 (mm/s²) */

osThreadId CarLockTaskHandle = NULL;
osThreadId CarTestTaskHandle = NULL;

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

    /* 连续发送3遍，增加短延时以保证总线稳定 */
    for (int i = 0; i < 3; i++) {
        CAN_SendData(0x316, buf, 8);
        osDelay(1);
    }
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

    /* 连续发送3遍，增加短延时以保证总线稳定 */
    for (int i = 0; i < 3; i++) {
        CAN_SendData(0x413, buf, 5);
        osDelay(1);
    }
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

/**
  * @brief  发送电机位移命令（内部函数）
  * @param  dev_id: 设备号
  * @param  displacement: 位移值（单位：mm，正数或负数）
  * @retval 无
  */
static void Car_SendDisplacement(uint8_t dev_id, float displacement)
{
    /* 将 mm 转换为 0.1mm 单位的整数（补码） */
    int32_t displacement_raw = (int32_t)(displacement * 10.0f);
    
    uint8_t cmd[6];
    cmd[0] = (uint8_t)(displacement_raw & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((displacement_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((displacement_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((displacement_raw >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    cmd[5] = 0x00;
    
    CAN_SendData(0x416, cmd, 6);
}

/**
  * @brief  小车前进
  * @param  distance: 前进距离（单位：mm，正数）
  * @retval 无
  * @note   前进：左轮负数位移，右轮正数位移
  */
void Car_MoveForward(float distance)
{
    /* 每次调用设置直线速度（使用全局变量，单位转换到 0.1mm/s） */
    uint32_t velocity = (uint32_t)(g_linear_velocity * 10.0f);
    Car_SetVelocity(current_left_dev, velocity);
    Car_SetVelocity(current_right_dev, velocity);

    /* 发送左、右位移命令各两遍，保留 1ms 延时 */
    for (int i = 0; i < 2; i++) {
        Car_SendDisplacement(current_left_dev, -distance);
        //osDelay(1);
        Car_SendDisplacement(current_right_dev, distance);
        osDelay(1);
    }
}

/**
  * @brief  小车后退
  * @param  distance: 后退距离（单位：mm，正数）
  * @retval 无
  * @note   后退：左轮正数位移，右轮负数位移
  */
void Car_MoveBackward(float distance)
{
    /* 每次调用设置直线速度（使用全局变量，单位转换到 0.1mm/s） */
    uint32_t velocity = (uint32_t)(g_linear_velocity * 10.0f);
    Car_SetVelocity(current_left_dev, velocity);
    Car_SetVelocity(current_right_dev, velocity);

    /* 发送左、右位移命令各两遍，保留 1ms 延时 */
    for (int i = 0; i < 2; i++) {
        Car_SendDisplacement(current_left_dev, distance);
        osDelay(1);
        Car_SendDisplacement(current_right_dev, -distance);
        osDelay(1);
    }
}

/**
  * @brief  小车左转
  * @param  distance: 转动距离（单位：mm，正数）
  * @retval 无
  * @note   左转：两个轮子都是负数位移
  */
void Car_TurnLeft(float distance)
{
    /* 每次调用设置旋转速度（使用全局变量，单位转换到 0.1mm/s） */
    uint32_t velocity = (uint32_t)(g_rotation_velocity * 10.0f);
    Car_SetVelocity(current_left_dev, velocity);
    Car_SetVelocity(current_right_dev, velocity);

    /* 发送左右位移命令各两遍以触发旋转，保留 1ms 延时 */
    for (int i = 0; i < 2; i++) {
        Car_SendDisplacement(current_left_dev, -distance);
        //osDelay(1);
        Car_SendDisplacement(current_right_dev, -distance);
        osDelay(1);
    }
}

/**
  * @brief  小车右转
  * @param  distance: 转动距离（单位：mm，正数）
  * @retval 无
  * @note   右转：两个轮子都是正数位移
  */
void Car_TurnRight(float distance)
{
    /* 每次调用设置旋转速度（使用全局变量，单位转换到 0.1mm/s） */
    uint32_t velocity = (uint32_t)(g_rotation_velocity * 10.0f);
    Car_SetVelocity(current_left_dev, velocity);
    Car_SetVelocity(current_right_dev, velocity);

    /* 发送左右位移命令各两遍以触发旋转，保留 1ms 延时 */
    for (int i = 0; i < 2; i++) {
        Car_SendDisplacement(current_left_dev, distance);
        //osDelay(1);
        Car_SendDisplacement(current_right_dev, distance);
        osDelay(1);
    }
}

/**
  * @brief  小车停止
  * @param  无
  * @retval 无
  * @note   停止：发送位移为 0.1mm
  */
void Car_Stop(void)
{
    /* 改为通过发送位移 0.1mm 来停止电机，发送两遍并保留短延时以提高可靠性 */
    for (int i = 0; i < 2; i++) {
        Car_SendDisplacement(current_left_dev, 0.1f);
        //osDelay(1);
        Car_SendDisplacement(current_right_dev, 0.1f);
        osDelay(1);
    }
}

/**
  * @brief  设置轮径
  * @param  dev_id: 设备号
  * @param  diameter: 轮径（单位：0.01mm）
  * @retval 无
  */
void Car_SetWheelDiameter(uint8_t dev_id, uint16_t diameter)
{
    uint8_t cmd[4];
    cmd[0] = dev_id;
    cmd[1] = 0x24;
    cmd[2] = (uint8_t)(diameter & 0xFF);        /* 低字节 */
    cmd[3] = (uint8_t)((diameter >> 8) & 0xFF); /* 高字节 */
    
    /* 连续发送3遍 */
    for (int i = 0; i < 3; i++)
    {
        CAN_SendData(0x408, cmd, 4);
        osDelay(1);
    }
}

/**
  * @brief  设置加速度
  * @param  dev_id: 设备号
  * @param  acceleration: 加速度（单位：0.1mm/s²）
  * @retval 无
  */
void Car_SetAcceleration(uint8_t dev_id, uint32_t acceleration)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(acceleration & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((acceleration >> 8) & 0xFF);
    cmd[2] = (uint8_t)((acceleration >> 16) & 0xFF);
    cmd[3] = (uint8_t)((acceleration >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    
    /* 连续发送3遍 */
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x414, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置速度
  * @param  dev_id: 设备号
  * @param  velocity: 速度（单位：0.1mm/s）
  * @retval 无
  */
void Car_SetVelocity(uint8_t dev_id, uint32_t velocity)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(velocity & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((velocity >> 8) & 0xFF);
    cmd[2] = (uint8_t)((velocity >> 16) & 0xFF);
    cmd[3] = (uint8_t)((velocity >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    
    /* 连续发送3遍 */
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x415, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置从位移
  * @param  dev_id: 设备号
  * @param  displacement: 从位移（单位：mm，正数或负数）
  * @retval 无
  */
void Car_SetSlaveDisplacement(uint8_t dev_id, float displacement)
{
    /* 将 mm 转换为 0.1mm 单位的整数（补码） */
    int32_t displacement_raw = (int32_t)(displacement * 10.0f);
    
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(displacement_raw & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((displacement_raw >> 8) & 0xFF);
    cmd[2] = (uint8_t)((displacement_raw >> 16) & 0xFF);
    cmd[3] = (uint8_t)((displacement_raw >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    
    CAN_SendData(0x419, cmd, 5);
}

/**
  * @brief  设置从速度
  * @param  dev_id: 设备号
  * @param  velocity: 从速度（单位：0.1mm/s）
  * @retval 无
  */
void Car_SetSlaveVelocity(uint8_t dev_id, uint32_t velocity)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(velocity & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((velocity >> 8) & 0xFF);
    cmd[2] = (uint8_t)((velocity >> 16) & 0xFF);
    cmd[3] = (uint8_t)((velocity >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    
    /* 连续发送2遍 */
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x418, cmd, 5);
        osDelay(1);
    }
}

/**
  * @brief  设置从加速度
  * @param  dev_id: 设备号
  * @param  acceleration: 从加速度（单位：0.1mm/s²）
  * @retval 无
  */
void Car_SetSlaveAcceleration(uint8_t dev_id, uint32_t acceleration)
{
    uint8_t cmd[5];
    cmd[0] = (uint8_t)(acceleration & 0xFF);         /* 低字节 */
    cmd[1] = (uint8_t)((acceleration >> 8) & 0xFF);
    cmd[2] = (uint8_t)((acceleration >> 16) & 0xFF);
    cmd[3] = (uint8_t)((acceleration >> 24) & 0xFF); /* 高字节 */
    cmd[4] = dev_id;
    
    /* 连续发送2遍 */
    for (int i = 0; i < 2; i++)
    {
        CAN_SendData(0x417, cmd, 5);
        osDelay(1);
    }
}

/* 任务实现已移至 NewGyroscope.c */
