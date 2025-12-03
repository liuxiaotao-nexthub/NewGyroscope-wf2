#ifndef __TIM_H
#define __TIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void TIM2_Init(void);
float TIM_GetAngle(void);

#ifdef __cplusplus
}
#endif

#endif
