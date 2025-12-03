#include "can.h"
#include "xv7001bb.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <stdint.h>
#include "tim.h"

/* CAN 发送任务实现：每10ms读取传感器并发送状态、温度、角速度和角度 */
void CAN_SendTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        /* 读取状态寄存器并发送 (ID 0x001) */
        uint8_t status = XV7001BB_ReadReg(0x04);
        CAN_SendData(0x003, &status, 1);

        /* 读取温度并发送 (ID 0x002)
           温度按 *100 转为定点，发送格式: 1 字节符号, 2 字节大端 */
        float temp = XV7001BB_ReadTemperature();
        int32_t temp_fp = (int32_t)(temp * 100.0f);
        uint16_t temp_abs = (uint16_t)((temp_fp < 0) ? (uint32_t)(-temp_fp) : (uint32_t)temp_fp);
        uint8_t tempBuf[3];
        tempBuf[0] = (temp_fp < 0) ? 1 : 0;
        tempBuf[1] = (uint8_t)((temp_abs >> 8) & 0xFF);
        tempBuf[2] = (uint8_t)(temp_abs & 0xFF);
        CAN_SendData(0x001, tempBuf, 3);

        /* 读取角速度并发送 (ID 0x003)
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

/* CAN 控制任务：接收 ID 0x204 的低功耗控制命令 */
void CAN_ControlTask(void const *argument)
{
    (void)argument;
    for (;;)
    {
        if (CAN_RxFlag)
        {
            CAN_RxFlag = 0; /* 清除标志 */

            if (CAN_RxStdId == 0x204 && CAN_RxDataLen > 0)
            {
                uint8_t cmd = CAN_RxData[0];
                switch (cmd)
                {
                    case 0xAA:
                        /* 进入睡眠 */
                        enter_sleep_mode();
                        break;
                    case 0xBB:
                        /* 进入待机 */
                        enter_standby_mode();
                        break;
                    case 0xCC:
                        /* 退出低功耗 */
                        wake_up_gyro();
                        break;
                    default:
                        break;
                }
            }
        }
        osDelay(10);
    }
}
