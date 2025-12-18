#include "car.h"
#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <string.h>

/* 该文件作为小车底层驱动接口，任务实现已移至 NewGyroscope.c，后续在此添加硬件相关实现 */

/* 小车任务句柄（在 main 中创建） */
osThreadId CarLockTaskHandle = NULL;

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

/* 任务实现已移至 NewGyroscope.c */
