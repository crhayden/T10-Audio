/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "AudioClips/WarningContemporary.h"
#include "AudioClips/PwrOnConcise.h"
#include "AudioClips/NotificationSharp.h"
#include "AudioClips/AlarmNerdy.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  WARNING,
  ALARM,
  POWER_ON,
  NOTIFICATION,
} audio_clips_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX_DMA_VAL (uint32_t)(pow(2,16)-1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

osThreadId buttonTaskHandle;
osThreadId audioTaskHandle;
osMessageQId audioQueueHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
void ButtonTask(void const * argument);
void AudioTask(void const * argument);

/* USER CODE BEGIN PFP */
static  uint32_t _ReadPins();
static  void     _Play(uint16_t* pBuf, uint32_t len);
static  void     _PlayClip(audio_clips_t clip);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t _ReadPins() {
  uint32_t  res =   0;
  res       =   HAL_GPIO_ReadPin(GPIOE, SW5_Pin)  <<  2;
  res       |=  HAL_GPIO_ReadPin(GPIOE, SW6_Pin)  <<  1;
  res       |=  HAL_GPIO_ReadPin(GPIOA, Trigger_Pin);
  return  res;
}
static void _Play(uint16_t* pBuf, uint32_t len) {
  uint8_t   k = 0;
  uint8_t   i = len/MAX_DMA_VAL;
  //
  // assume audio clip is larger than max size (2^16)
  //
  do {
    //
    // Advance to start of next chunk 
    //
    uint16_t* pData = &pBuf[k * MAX_DMA_VAL];
    //
    // Get the length of current chunk
    //
    uint32_t l        = len - MAX_DMA_VAL * k;
    uint32_t clipSize = MIN(l, MAX_DMA_VAL);
    //
    // make sure we are ready before transmitting
    //
    while (HAL_I2S_GetState(&hi2s3) != HAL_I2S_STATE_READY);
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)pData, clipSize);
    k++;
  } while (i-->0);

}
static void _PlayClip(audio_clips_t clip) {
    switch (clip) {
        case WARNING:
          _Play((uint16_t*)&WarningContemporary[0], sizeWarningContemporary);
          break;
        case ALARM:
          _Play((uint16_t*)&AlarmNerdy[0], sizeAlarmNerdy);
          break;
        case POWER_ON:
          _Play((uint16_t*)&PwrOnConcise[0], sizePwrOnConcise);
          break;
        case NOTIFICATION:
          _Play((uint16_t*)&NotificationSharp[0], sizeNotificationSharp);
          break;
    default:
      break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of audioQueue */
  osMessageQDef(audioQueue, 16, uint16_t);
  audioQueueHandle = osMessageCreate(osMessageQ(audioQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, ButtonTask, osPriorityNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of audioTask */
  osThreadDef(audioTask, AudioTask, osPriorityIdle, 0, 128);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : SW5_Pin SW6_Pin */
  GPIO_InitStruct.Pin = SW5_Pin|SW6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigger_Pin */
  GPIO_InitStruct.Pin = Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trigger_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MCLK_DISABLED_Pin */
  GPIO_InitStruct.Pin = MCLK_DISABLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCLK_DISABLED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_ButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ButtonTask */
void ButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  uint32_t	curPinVal	=	 0;
  uint32_t 	oldVal 		=  -1;
  for(;;) {
	curPinVal	=	_ReadPins();
	//
	// button release is required in between every playback iteration
	//
	if (curPinVal != oldVal) {
		oldVal	=	curPinVal;
		//
		// Only attempt to play clip if there isn't one already playing
		//
		if (HAL_I2S_GetState(&hi2s3) == HAL_I2S_STATE_READY) {
			switch (curPinVal) {
  				case 3:
  					osMessagePut (audioQueueHandle, WARNING, 100);
  					break;
  				case 5:
  					osMessagePut (audioQueueHandle, NOTIFICATION, 100);
  					break;
  				case 6:
  					osMessagePut (audioQueueHandle, ALARM, 100);
  					break;
  				default:
  					break;
			}
		}
	}
  osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_AudioTask */
/**
* @brief Function implementing the audioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AudioTask */
void AudioTask(void const * argument)
{
  /* USER CODE BEGIN AudioTask */
  /* Infinite loop */
  osEvent event;
  audio_clips_t clip = 0;
  _PlayClip(POWER_ON);
  for(;;)
  {
	  event = osMessageGet(audioQueueHandle, osWaitForever);
	  if (event.status == osEventMessage) {
		  event.def.message_id  = audioQueueHandle;
		  clip 					        = (audio_clips_t)event.value.v;
      _PlayClip(clip);
	  }
	  osDelay(100);
  }
  /* USER CODE END AudioTask */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
