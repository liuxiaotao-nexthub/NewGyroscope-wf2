/**
  ******************************************************************************
  * @file    xv7001bb.c
  * @author  应用团队
  * @brief   XV7001BB陀螺仪驱动实现
  ******************************************************************************
  */

/* 包含文件 -------------------------------------------------------------------*/
#include "xv7001bb.h"
#include "cmsis_os.h"

/* 私有函数 -------------------------------------------------------------------*/

/**
  * @brief  初始化XV7001BB - 仅取消I²C功能
  * @param  无
  * @retval 无
  */
void XV7001BB_Init(void)
{
	
	/* 使用 HAL 延时 2 ms */
	HAL_Delay(300);
	
    /* 向寄存器0x1F写入0x00，禁用I²C */
    XV7001BB_WriteReg(0x1F, 0x00);


    /* 设置内部低通滤波器为 4 阶 50Hz
       寄存器 0x02: bit5..4 = LpfOrder (10 = 4阶), bit3..0 = LpfFc (0011 = 50Hz)
       0x02 = 0010_0011 = 0x23
    */
    XV7001BB_WriteReg(0x02, 0x22);

    /* 执行 DSP 复位以应用滤波器设置 (命令寄存器 0x0d) */
    XV7001BB_WriteReg(0x0d, 0x00);

    /* 延时等待 DSP 复位完成 */
    HAL_Delay(5);
}

/**
  * @brief  写寄存器
  * @param  reg: 寄存器地址
  * @param  data: 数据
  * @retval 无
  */
void XV7001BB_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    SPI_CS_LOW();
    SPI_Transmit(buf, 2);
    SPI_CS_HIGH();
}

/**
  * @brief  读寄存器
  * @param  reg: 寄存器地址
  * @retval 寄存器值
  */
uint8_t XV7001BB_ReadReg(uint8_t reg)
{
    uint8_t addr = reg | 0x80; /* 读命令 */
    uint8_t data = 0;

    SPI_CS_LOW();
    /* 先发送寄存器地址（读命令），再接收一个字节数据 */
    SPI_Transmit(&addr, 1);
    SPI_Receive(&data, 1);
    SPI_CS_HIGH();

    return data;
}

/**
  * @brief  进入待机模式
  * @param  无
  * @retval 无
  */
void enter_standby_mode(void) {
    XV7001BB_WriteReg(CMD_STANDBY, 0x00);
}

/**
  * @brief  进入睡眠模式
  * @param  无
  * @retval 无
  */
void enter_sleep_mode(void) {
    XV7001BB_WriteReg(CMD_SLEEP_IN, 0x00);
}

/**
  * @brief  退出低功耗模式
  * @param  无
  * @retval 无
  */
void wake_up_gyro(void) {
    XV7001BB_WriteReg(CMD_SLEEP_OUT, 0x00);
    HAL_Delay(200); /* 等待 tSTA */
}

/**
  * @brief  读取温度，默认12位，返回摄氏度
  * @param  无
  * @retval 温度值
  */
float XV7001BB_ReadTemperature(void)
{
    uint8_t tx[2];
    uint8_t rx[2];
    int16_t raw;
    SPI_StatusTypeDef status;

    tx[0] = 0x08 | 0x80; /* 读温度寄存器 */

    SPI_CS_LOW();
    /* 发送寄存器地址，然后接收两个字节 */
    status = SPI_Transmit(&tx[0], 1);
    (void)status;
    status = SPI_Receive(rx, 2);
    (void)status;
    SPI_CS_HIGH();

    /* 12位模式: rx[0]=D[11:4], rx[1]=D[3:0]<<4 */
    raw = (rx[0] << 4) | (rx[1] >> 4);
    /* 符号扩展 */
    if (raw & 0x0800) raw |= 0xF000;

    return (float)raw / 16.0f; /* 12位: 16 LSB/°C */
}

/**
  * @brief  读取角速度，默认16位，返回 °/s
  * @param  无
  * @retval 角速度值
  */
float XV7001BB_ReadAngularRate(void)
{
    uint8_t tx[2];
    uint8_t rx[2];
    int16_t raw;
    SPI_StatusTypeDef status;

    tx[0] = 0x0A | 0x80; /* 读角速度寄存器 */

    SPI_CS_LOW();
    /* 发送寄存器地址，然后接收两个字节 */
    status = SPI_Transmit(&tx[0], 1);
    (void)status;
    status = SPI_Receive(rx, 2);
    (void)status;
    SPI_CS_HIGH();

    raw = (rx[0] << 8) | rx[1];

    return (float)raw / 70.0f; /* 16位灵敏度: 70 LSB/(°/s) */
}

/************************ 文件结束 ****/
