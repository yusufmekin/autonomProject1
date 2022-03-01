/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	LED_ON = 0,
	LED_OFF,
	LED_1S
}led_state;

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t rxTempBuff[1];
uint8_t rxBuff[50];
uint8_t txBuff[50];

uint16_t msTick=0;		// ms cinsinden sayım yapacak değişken		1000
uint8_t sec; 			// sn cinsinden sayım yapacak değişken		  1

uint16_t controlOnTime = 0;
uint16_t controlOffTime;

uint8_t rxIndex = 0;
uint8_t txBuffLength;
uint8_t uartSuccessCounter;
uint8_t newMessageArrive = 0;

bool echoTaskWorkingState = false;

uint16_t stopZamanSayaci = 1000;
uint16_t ledOnZamanSayaci = 0;
uint16_t ledOffZamanSayaci = 0;
uint16_t gelenZaman;
uint8_t waitLedOff = 0;
uint8_t waitLedOffCounter;
uint8_t waitLedOn = 0;
uint8_t waitLedOnCounter;
uint8_t ledState;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)					// Uart interrupt function aktif edildi.
{
	if(huart == &huart2)
	{
		rxBuff[rxIndex] = rxTempBuff[0];

		if(rxBuff[rxIndex] == '\n')
		{
			for(int i = 0; i <= rxIndex; i++)
				txBuff[i] = rxBuff[i];									// Echo için txBuff'a alındı.

			newMessageArrive = 1;

			for(int i = 0; i <= rxIndex; i++)
				rxBuff[i] = 0;

			txBuffLength = rxIndex + 1;

			rxIndex = 0;
		}
		else
		{
			rxIndex++;
		}

		HAL_UART_Receive_IT(&huart2, rxTempBuff, 1);
	}
}


// Bir dizi alacağım için dizinin ismini buraya girdiğimde aslında dizinin adresini vermiş olacağım.
void UART_Echo_Task(uint8_t* receiveMesage, uint8_t receiveMessageLength)
{
	if(newMessageArrive)
	{
		HAL_UART_Transmit_IT(&huart2, receiveMesage, receiveMessageLength);

		newMessageArrive = 0;
	}
	else
	{

	}
}

void LED_Control_Task(uint8_t state, uint32_t time)
{
	if(state == LED_ON)
	{
		// Önce led yakma işlemi yapılacak		-> time süresi boyunca led yanacak
		// Sonra led söndürme işlemi yapılacak	-> 1000-time süresi boyunca led sönük kalacak

		if(ledOnZamanSayaci == 0 && waitLedOff == 0)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			ledOffZamanSayaci = 0;
			ledOnZamanSayaci = time;
			waitLedOff = 1;
		}

		if(ledOnZamanSayaci < 1)
		{
			ledOnZamanSayaci = 0;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

			if(ledOffZamanSayaci < 1)
			{
				ledOffZamanSayaci = 1000 - time;

				waitLedOffCounter++;

				if(waitLedOffCounter > 1)
				{
					waitLedOff = 0;
					waitLedOffCounter = 0;
				}
			}
		}

	}
	else if(state == LED_OFF)
	{
		if(ledOffZamanSayaci == 0 && waitLedOn == 0)
		{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			ledOnZamanSayaci = 0;
			ledOffZamanSayaci = time;
			waitLedOn = 1;
		}

		if(ledOffZamanSayaci < 1)
		{
			ledOffZamanSayaci = 0;
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

			if(ledOnZamanSayaci < 1)
			{
				ledOnZamanSayaci = 1000 - time;
				waitLedOnCounter++;

				if(waitLedOnCounter > 1)
				{
					waitLedOn = 0;
					waitLedOnCounter = 0;
				}
			}
		}
	}
	else	// STOP -> LED 1sn
	{
		if(stopZamanSayaci < 1)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
			stopZamanSayaci = 1000;
		}
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(txBuff[0] == 's' && txBuff[1] == 't' && txBuff[2] == 'a' && txBuff[3] == 'r' && txBuff[4] == 't') 			// döngüye girmesi gereken komutlar eklendi ve testler yapıldı.
	  {
		  echoTaskWorkingState = true;
	  }
	  else if(txBuff[0] == 's' && txBuff[1] == 't' && txBuff[2] == 'o' && txBuff[3] == 'p')
	  {
		  echoTaskWorkingState = false;
		  ledState = LED_1S;
	  }
	  else
	  {

	  }

	  if(txBuff[0] == 'l' && txBuff[1] == 'e' && txBuff[2] == 'd' && txBuff[3] == 'o' && txBuff[4] == 'n' && txBuff[5] == '=')
	  {
		  //txBuff[6];	// yüzler basamağı
		  //txBuff[7];	// onlar basamağı
		  //txBuff[8];	// birler basamağı

		  gelenZaman = (txBuff[6] - 48) * 100 + (txBuff[7] - 48) * 10 + (txBuff[8] - 48) * 1;
		  ledState = LED_ON;
	  }

	  if(txBuff[0] == 'l' && txBuff[1] == 'e' && txBuff[2] == 'd' && txBuff[3] == 'o' && txBuff[4] == 'f' && txBuff[5] == 'f' && txBuff[6] == '=')
	  {
		  //txBuff[6];	// yüzler basamağı
		  //txBuff[7];	// onlar basamağı
		  //txBuff[8];	// birler basamağı

		  gelenZaman = (txBuff[7] - 48) * 100 + (txBuff[8] - 48) * 10 + (txBuff[9] - 48) * 1;
		  ledState = LED_OFF;
	  }

	  if(echoTaskWorkingState == true)
	  {
		  UART_Echo_Task(txBuff, txBuffLength);
		  LED_Control_Task(ledState, gelenZaman);
	  }
	  else
	  {
		  LED_Control_Task(ledState, 1000);
	  }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

