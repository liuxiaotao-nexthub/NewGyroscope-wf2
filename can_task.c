#include "can.h"
#include "xv7001bb.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <stdint.h>
#include <string.h>
#include "tim.h"

/* 外部变量（小车队列句柄，在 NewGyroscope.c 中定义） */
extern osMessageQId CarCanQueueHandle;

/* CAN 发送任务实现：每10ms读取传感器并发送状态、温度、角速度和角度 */
void CAN_SendTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        /* 读取状态寄存器并发送 (ID 0x003) */
        uint8_t status = XV7001BB_ReadReg(0x04);
        CAN_SendData(0x003, &status, 1);

        /* 读取温度并发送 (ID 0x001)
           温度按 *100 转为定点，发送格式: 1 字节符号, 2 字节大端 */
        float temp = XV7001BB_ReadTemperature();
        int32_t temp_fp = (int32_t)(temp * 100.0f);
        uint16_t temp_abs = (uint16_t)((temp_fp < 0) ? (uint32_t)(-temp_fp) : (uint32_t)temp_fp);
        uint8_t tempBuf[3];
        tempBuf[0] = (temp_fp < 0) ? 1 : 0;
        tempBuf[1] = (uint8_t)((temp_abs >> 8) & 0xFF);
        tempBuf[2] = (uint8_t)(temp_abs & 0xFF);
        CAN_SendData(0x001, tempBuf, 3);

        /* 读取角速度并发送 (ID 0x002)
           角速度按 *10000 转为定点，发送格式: 1 字节符号, 3 字节大端 */
        float rate = XV7001BB_ReadAngularRate();
        int32_t rate_fp = (int32_t)(rate * 10000.0f);
        uint32_t rate_abs = (rate_fp < 0) ? (uint32_t)(-rate_fp) : (uint32_t)rate_fp;
        uint8_t txRate[4];
        txRate[0] = (rate_fp < 0) ? 1 : 0;
        txRate[1] = (uint8_t)((rate_abs >> 16) & 0xFF);
        txRate[2] = (uint8_t)((rate_abs >> 8) & 0xFF);
        txRate[3] = (uint8_t)(rate_abs & 0xFF);
        CAN_SendData(0x002, txRate, 4);

        /* 读取积分角度并发送 (ID 0x004)
           角度按 *10000 转为定点，发送格式: 1 字节符号, 3 字节大端 */
        float angle = TIM_GetAngle();
        int32_t angle_fp = (int32_t)(angle * 10000.0f);
        uint32_t angle_abs = (angle_fp < 0) ? (uint32_t)(-angle_fp) : (uint32_t)angle_fp;
        uint8_t txAngle[4];
        txAngle[0] = (angle_fp < 0) ? 1 : 0;
        txAngle[1] = (uint8_t)((angle_abs >> 16) & 0xFF);
        txAngle[2] = (uint8_t)((angle_abs >> 8) & 0xFF);
        txAngle[3] = (uint8_t)(angle_abs & 0xFF);
        CAN_SendData(0x004, txAngle, 4);

        osDelay(10);
    }
}

/**
  * @brief  CAN 控制任务：接收 CAN 数据并分发到对应处理
  * @param  argument 未使用
  * @retval 无
  */
void CAN_ControlTask(void const *argument)
{
    (void)argument;

    /* 小车队列消息结构 */
    typedef struct {
        uint32_t id;
        uint8_t data[8];
        uint8_t len;
    } CarCanMsg_t;

    CarCanMsg_t carMsg;

    for (;;)
    {
        if (CAN_RxFlag)
        {
            /* 复制数据到本地以避免并发问题 */
            uint32_t id = CAN_RxStdId;
            uint8_t len = (uint8_t)CAN_RxDataLen;
            uint8_t localData[8];
            memcpy(localData, CAN_RxData, len);

            /* 清除全局接收标志 */
            CAN_RxFlag = 0;

            /* 处理 0x312（SN 上报）和 0x409（电机位移）消息 */
            if ((id == 0x312 && len == 7) || (id == 0x409 && len == 8))
            {
                /* 将消息放入队列，供 Car_LockTask 处理 */
                carMsg.id = id;
                carMsg.len = len;
                memcpy(carMsg.data, localData, len);
                osMessagePut(CarCanQueueHandle, (uint32_t)&carMsg, 0);
            }
        }

        osDelay(10);
    }
}
