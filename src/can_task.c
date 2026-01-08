#include "can.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include "queue.h"
#include <stdint.h>
#include <string.h>

/* 外部变量（小车队列句柄，在 NewGyroscope.c 中定义） */
extern osMessageQId CarCanQueueHandle;

/* CAN 发送任务实现 */
void CAN_SendTask(void const *argument)
{
    (void)argument;

    for (;;)
    {
        /* 可在此添加 CAN 发送逻辑 */
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

            /* 处理 0x312（SN 上报）、0x409（电机位移）以及 0x420+dev（剩余位移）消息 */
            if ((id == 0x312 && len == 7) || (id == 0x409) || (id >= 0x421 && id <= 0x42F))
            {
                /* 将消息放入队列 */
                carMsg.id = id;
                carMsg.len = len;
                memcpy(carMsg.data, localData, len);
                /* 使用 xQueueOverwrite 覆盖旧数据（队列大小为1时适用） */
                xQueueOverwrite(CarCanQueueHandle, &carMsg);
            }
        }

        osDelay(5);
    }
}
