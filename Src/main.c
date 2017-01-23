/**
  ******************************************************************************
  * @file    TIM/TIM_DMA/Src/main.c
  * @author  MCD Application Team
  * @version V1.6.0
  * @date    01-July-2016
  * @brief   This sample code shows how to use DMA with TIM2 Update request to
  *          transfer Data from memory to TIM2 Capture Compare Register 3 (CCR3).
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"



/** @addtogroup STM32L1xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_DMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */
TIM_HandleTypeDef    TimHandle;

/* Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Timer Period*/
uint32_t uwTimerPeriod  = 0;


UART_HandleTypeDef UartHandle;

__IO ITStatus UartReady = RESET;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA**** ";
/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE1];

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
uint32_t WS2812_IO_framedata[24*40+100];
int dlzka=24*40+100;

//========================================================================================//
// Nepouzita funkcia
//
// Zmysel je v meneni danej ledky na danej pozicii
// Irelevantne v ramci fubjcie programu(posielame cely retazec)
//========================================================================================//
void color2pwmMatrix(int row, int column, uint8_t color1, uint8_t color2,uint8_t color3) {
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 8; j++) {
			if (row == i && column == j) {

				color2pwm((i * 8 + j), color1, color2, color3);
			}

		}

	}

}

//========================================================================================//
// Funkcia len na cakanie nech sa vsetko stihne udiat
//========================================================================================//
void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

//========================================================================================//
// Hlavna funkcia prevadzania priateho buffra na farby do buffra ktory sa posiela
//========================================================================================//
void color2pwm(int pos, uint8_t color1, uint8_t color2, uint8_t color3) {
	uint8_t mask;
	int j,pwm_pos = pos*24;
	mask = 0x80;

	if(color1 > 255)
		color1 = 255;
	if(color2 > 255)
		color2 = 255;
	if(color3 > 255)
		color3 = 255;


	for (j = 0; j < 8; j++)
	{
		if ( (color1<<j) & 0x80 )
		{
			WS2812_IO_framedata[pwm_pos+j] =(uint32_t)(((uint32_t) 66 * (uwTimerPeriod - 1)) / 100);
; 	// compare value for logical 1
		}
		else
		{
			WS2812_IO_framedata[pwm_pos+j] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);// compare value for logical 0
		}

		if ( (color2<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
		{
			WS2812_IO_framedata[pwm_pos+8+j] = (uint32_t)(((uint32_t) 66 * (uwTimerPeriod - 1)) / 100); 	// compare value for logical 1
		}
		else
		{
			WS2812_IO_framedata[pwm_pos+8+j] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);	// compare value for logical 0
		}

		if ( (color3<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
		{
			WS2812_IO_framedata[pwm_pos+16+j] = (uint32_t)(((uint32_t) 66 * (uwTimerPeriod - 1)) / 100);	// compare value for logical 1   15
		}
		else
		{
			WS2812_IO_framedata[pwm_pos+16+j] = (uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);	// compare value for logical 0     6
		}
	}
}

//========================================================================================//
// Prijaty char na cislo
//
// Po dvojici
//========================================================================================//
uint8_t charToNumJa(uint8_t ityChar[2])
{
	int cislo1;

	if (ityChar[0] < 'A')
		cislo1 = (ityChar[0] - '0') * 16;
	else
		cislo1 = (ityChar[0] - 'A' + 10) * 16;

	if (ityChar[1] < 'A')
		cislo1 += (ityChar[1] - '0');
	else
		cislo1 += (ityChar[1] - 'A' + 10);

	return cislo1;
}

//========================================================================================//
// Main
//========================================================================================//
int main(void)
{
 /* This sample code shows how to use DMA with TIM2 Update request to transfer
  Data from memory to TIM2 Capture Compare Register 3 (CCR3), through the
  STM32L1xx HAL API. To proceed, 3 steps are required */

  /* STM32L1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */

  //Inicializacie

  HAL_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);
  BSP_PB_Init(BUTTON_USER,BUTTON_MODE_EXTI);

  /* Configure the system clock to 32 MHz */
  SystemClock_Config();

  /* Compute the value of ARR regiter to generate signal frequency at 800 Khz */
  uwTimerPeriod = (uint32_t)((SystemCoreClock / 800000) - 1);


  for(int i=0;i<(24*40+100);i++)
  {
		WS2812_IO_framedata[i]=(uint32_t)(((uint32_t) 33 * (uwTimerPeriod - 1)) / 100);
 	}

  for(int i=0;i<100;i++)
  {
		WS2812_IO_framedata[24*40+i]=0;
	}
  /*##-1- Configure the TIM peripheral #######################################*/
  /* ---------------------------------------------------------------------------
  TIM1 input clock (TIM2CLK) is set to APB1 clock (PCLK1), since APB1
  prescaler is 1.
    TIM2CLK = PCLK1
    PCLK1 = HCLK
    => TIM2CLK = HCLK = SystemCoreClock

  TIM2CLK = SystemCoreClock, Prescaler = 0, TIM2 counter clock = SystemCoreClock
  SystemCoreClock is set to 32 MHz for STM32L1xx devices.

  The objective is to configure TIM2 channel 3 to generate a PWM
  signal with a frequency equal to 17.57 KHz:
     - TIM2_Period = (SystemCoreClock / 17570) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32l1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  -----------------------------------------------------------------------------*/
  /* Initialize TIM2 peripheral as follows:
      + Period = TimerPeriod (To have an output frequency equal to 17.570 KHz)
      + Prescaler = 0
      + ClockDivision = 0
      + Counter direction = Up
  */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE1)!= HAL_OK)
  {
    Error_Handler();
  }

  /*##-3- Start the transmission process #####################################*/
  /* While the UART in reception process, user can transmit data through
     "aTxBuffer" buffer */
  if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
  {
    Error_Handler();
  }


  TimHandle.Instance = TIMx;

  TimHandle.Init.Period            = uwTimerPeriod;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channel 3 ########################################*/
  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.Pulse        = aCCValue_Buffer[0];
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-3- Start PWM signal generation in DMA mode ############################*/
  if (HAL_TIM_PWM_Start_DMA(&TimHandle, TIM_CHANNEL_3, WS2812_IO_framedata, dlzka) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  while (1)
  {
    // Zablokuje prerusenie po prijati, vsetko vynuluje pripravi na dalsie prijatie
    HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer,RXBUFFERSIZE1 );

    // Cakame nech moc nenahlime na dosku zbytcne
    Delay(500000L);
  }

}

//==========================================================//
// V pripade niakej chyby zazne LED2
//==========================================================//
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}


//==========================================================//
// Inicializacia systemoveho casu
//==========================================================//
  /*
  * @retval None
  */

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

//========================================================================================//
// Uart prerusenie 
//========================================================================================//
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: trasfer complete*/
  UartReady = SET;
  uint8_t ulzene1[2];
  uint8_t ulzene2[2];
  uint8_t ulzene3[2];


	for(int i=0;i<40;i ++)
    {
      // Rozobratie stringu
  	 		strncpy(ulzene1, aRxBuffer + i*6, 2);
  	 		strncpy(ulzene2, aRxBuffer + (i*6 + 2), 2);
  	 		strncpy(ulzene3, aRxBuffer + (i*6 + 4), 2);

      // Zmena prijateho retazca na retazec farieb
  	 		color2pwm(i, charToNumJa(ulzene2), charToNumJa(ulzene1), charToNumJa(ulzene3));
	 	}
    // Zablokuje prerusenie po prijati, vsetko vynuluje pripravi na dalsie prijatie
	  HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer,RXBUFFERSIZE1 );
}

//========================================================================================//
// Prazdny error handler
//========================================================================================//
/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}

//========================================================================================//
// Hal funkc
//========================================================================================//
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == USER_BUTTON_PIN)
  {
    UserButtonStatus = 1;
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
