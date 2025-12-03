/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "spi.h"
#include "can.h"
#include "xv7001bb.h"
#include "can_task.h"
#include "tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId MainTaskHandle;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Main_Task(void const *argument);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2 (APB1 = 36MHz)
  *            APB2 Prescaler                 = 1 (APB2 = 72MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
  *            PLL Multiplier                 = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		while (1);
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		while (1);
	}
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32F1xx HAL library initialization:
	     - Configure the Flash prefetch, instruction and Data caches
	     - Configure the Systick to generate an interrupt each 1 msec
	     - Set NVIC Group Priority to 4
	     - Global MSP (MCU Support Package) initialization
	   */
	HAL_Init();
	
	/* Configure the system clock to 72 MHz */
	SystemClock_Config();
	
	/* Enable GPIOB Clock */
	__GPIOB_CLK_ENABLE();
	
	/* Configure PB9 as output */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialize SPI */
	SPI_Init();

	/* Initialize CAN */
	CAN_Init();

	/* Initialize XV7001BB */
	XV7001BB_Init();

	/* Initialize TIM2 for 2ms integration interrupts */
	TIM2_Init();

	/* Create CAN control task */
	osThreadDef(CANCTL, CAN_ControlTask, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate(osThread(CANCTL), NULL);

	/* Create CAN send task */
	osThreadDef(CANSEND, CAN_SendTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadCreate(osThread(CANSEND), NULL);

	/* Main task definition */
	osThreadDef(MAIN, Main_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	
	/* Start main task */
	MainTaskHandle = osThreadCreate(osThread(MAIN), NULL);
	
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	for (;;)
		;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

/**
  * @brief  主任务 - 翻转PB9的LED
  * @param  argument 未使用
  * @retval 无
  */
static void Main_Task(void const *argument)
{
    (void) argument;
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
        osDelay(100);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
