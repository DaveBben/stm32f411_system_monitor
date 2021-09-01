/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "ws2812b/ws2812b.h"
#include <stdint.h>
#include <stdlib.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Helper defines
#define newColor(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) <<  8) | (b))
#define Red(c) ((uint8_t)((c >> 16) & 0xFF))
#define Green(c) ((uint8_t)((c >> 8) & 0xFF))
#define Blue(c) ((uint8_t)(c & 0xFF))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */
// RGB Framebuffers
uint8_t frameBuffer[3*60];
uint8_t frameBuffer2[3*20];



typedef enum {
	  	CPU_FREQ = 1,
	    CPU_UTIL = 2,
	    CPU_SPEED = 3,
	    CPU_TEMP = 4,
	    CPU_NAME = 5,
	    RAM = 6,
	    GPU_FREQ = 7,
	    GPU_SPEED = 8,
	    GPU_MEM = 9,
	    GPU_FPS = 10,
	    GPU_NAME = 11,
	    GPU_UTIL = 12,
	    GPU_TEMP = 13,

} DataHeaders;

struct System {
   int  cpu_frequency;
   uint8_t  cpu_util;
   uint8_t cpu_speed;
   uint8_t  cpu_temp;
   char   cpu_name[50];
   uint8_t ram_util;
   int gpu_freq;
   char gpu_speed[50];
   uint8_t gpu_mem;
   int gpu_fps;
   char gpu_name[50];
   uint8_t gpu_util;
   uint8_t gpu_temp;
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
extern uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
extern void CDC_ReadRxBuffer_FS(uint8_t *Buf, uint8_t Len);
extern uint8_t CDC_GetRxBufferBytesAvailable_FS(void);
extern void CDC_FlushRxBuffer_FS();
extern void CDC_Read_Next();
void Monitor_Get_Values(uint8_t *raw_string, size_t srcSize, int *buffer);
void writeToDisplay(char *str);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void writeToDisplay(char *str) {
	//HAL_Delay(1000);

	// ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);

	//HAL_Delay(1000);

	// Write data to local screenbuffer
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(str, Font_11x18, White);

//	     ssd1306_SetCursor(0, 36);
//	     ssd1306_WriteString(str, Font_11x18, White);

//	     // Draw rectangle on screen
//	     for (uint8_t i=0; i<28; i++) {
//	         for (uint8_t j=0; j<64; j++) {
//	             ssd1306_DrawPixel(100+i, 0+j, White);
//	         }
//	     }

// Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);

}

void updateDisplay(struct System *system) {



	// ssd1306_Fill(Black);
	ssd1306_UpdateScreen(&hi2c1);

	char cpu_text[20] = {0};

	sprintf(cpu_text, "CPU %d %d C", system->cpu_util, system->cpu_temp);




	// Write data to local screenbuffer
	ssd1306_SetCursor(0, 15);
	ssd1306_WriteString(cpu_text, Font_11x18, White);



// Copy all data from local screenbuffer to the screen
	ssd1306_UpdateScreen(&hi2c1);

}

uint32_t Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return newColor(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return newColor(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return newColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}



void visRainbow(uint8_t *frameBuffer, uint32_t frameBufferSize, uint32_t effectLength)
{
	uint32_t i;
	static uint8_t x = 0;

	x += 1;

	if(x == 256*5)
		x = 0;

	for( i = 0; i < frameBufferSize / 3; i++)
	{
		uint32_t color = Wheel(((i * 256) / effectLength + x) & 0xFF);

		frameBuffer[i*3 + 0] = color & 0xFF;
		frameBuffer[i*3 + 1] = color >> 8 & 0xFF;
		frameBuffer[i*3 + 2] = color >> 16 & 0xFF;
	}
}


void visDots(uint8_t *frameBuffer, uint32_t frameBufferSize, uint32_t random, uint32_t fadeOutFactor)
{
	uint32_t i;

	for( i = 0; i < frameBufferSize / 3; i++)
	{

		if(rand() % random == 0)
		{
			frameBuffer[i*3 + 0] = 255;
			frameBuffer[i*3 + 1] = 255;
			frameBuffer[i*3 + 2] = 255;
		}


		if(frameBuffer[i*3 + 0] > fadeOutFactor)
			frameBuffer[i*3 + 0] -= frameBuffer[i*3 + 0]/fadeOutFactor;
		else
			frameBuffer[i*3 + 0] = 0;

		if(frameBuffer[i*3 + 1] > fadeOutFactor)
			frameBuffer[i*3 + 1] -= frameBuffer[i*3 + 1]/fadeOutFactor;
		else
			frameBuffer[i*3 + 1] = 0;

		if(frameBuffer[i*3 + 2] > fadeOutFactor)
			frameBuffer[i*3 + 2] -= frameBuffer[i*3 + 2]/fadeOutFactor;
		else
			frameBuffer[i*3 + 2] = 0;
	}
}



void neopixel_handle2()
{
	static uint32_t timestamp;

	if(HAL_GetTick() - timestamp > 10)
	{
		timestamp = HAL_GetTick();

		// Animate next frame, each effect into each output RGB framebuffer
		visRainbow(frameBuffer, sizeof(frameBuffer), 15);
		visDots(frameBuffer2, sizeof(frameBuffer2), 50, 40);
	}
}



void neopixel_handle(){
	if(ws2812b.transferComplete)
		{
			// Update your framebuffer here or swap buffers
			neopixel_handle2();

			// Signal that buffer is changed and transfer new data
			ws2812b.startTransfer = 1;
			ws2812b_handle();
		}
}

void neopixel_init(){
	uint8_t i;



		// 4 paralel output LED strips needs 18% overhead during TX
		// 8 paralel output LED strips overhead is 8us of 30us period which is 28% - see the debug output PD15/13

		// If you need more parallel LED strips, increase the WS2812_BUFFER_COUNT value
		for( i = 0; i < WS2812_BUFFER_COUNT; i++)
		{

			// Set output channel/pin, GPIO_PIN_0 = 0, for GPIO_PIN_5 = 5 - this has to correspond to WS2812B_PINS
			ws2812b.item[i].channel = i;

			// Every even output line has second frameBuffer2 with different effect
			if(i % 2 == 0)
			{
				// Your RGB framebuffer
				ws2812b.item[i].frameBufferPointer = frameBuffer;
				// RAW size of framebuffer
				ws2812b.item[i].frameBufferSize = sizeof(frameBuffer);
			} else {
				ws2812b.item[i].frameBufferPointer = frameBuffer2;
				ws2812b.item[i].frameBufferSize = sizeof(frameBuffer2);
			}

		}


		ws2812b_init();
	}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char msg[11] = "No Data";

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	// Init lcd using one of the stm32HAL i2c typedefs
	if (ssd1306_Init(&hi2c1) != 0) {
		Error_Handler();
	}

//	writeToDisplay(msg);

	char message[20] = { 0 };

	uint8_t data_frame_buffer[64] = {0};

	struct System system;

	DataHeaders headers;


   // neopixel_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//neopixel_handle();

		uint8_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
		if (bytesAvailable != 0) {
			memset(data_frame_buffer, 0, 64);  // clear the buffer
			CDC_ReadRxBuffer_FS(data_frame_buffer, bytesAvailable);

			DataHeaders data_head = (DataHeaders) data_frame_buffer[0];


			switch(data_head){
			    case CPU_UTIL: {
			    	system.cpu_util = data_frame_buffer[1];
			    	break;
			    }
			    case CPU_TEMP: {
			    	system.cpu_temp = data_frame_buffer[1];
			    	break;
			    }
			    case RAM: {
			    	system.ram_util = data_frame_buffer[1];
			    	break;
			    }
			    default: {
			      break;
			    }
			}

			updateDisplay(&system);
			CDC_FlushRxBuffer_FS();
			CDC_Read_Next();



		}

//	  CDC_Transmit_FS(msg,sizeof(msg));
//	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
