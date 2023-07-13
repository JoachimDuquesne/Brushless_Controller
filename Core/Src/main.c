/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MIN_IDLE_TIME	5	// Minimum time at idle before starting the motor in seconds
#define MIN_STARTUP_TIME 10 // Minimum time for startup before increasing speed

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t _fw_mode = 0;
uint32_t _fw_status = 0;

float _fw_target_speed = 0;

const float fw_off_duty           = 0.02;
const float fw_idle_duty          = 0.28;
const float fw_min_duty           = 0.32;
const float fw_max_duty           = 0.95;

enum {
	FW_STATUS_OFF=0,
	FW_STATUS_IDLE,
	FW_STATUS_RUNNING,
};



uint8_t UARTcmd_txBuffer[256] = {0};
uint8_t UARTcmd_rxBuffer[64] = {0};
uint8_t error=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void set_LED_Blink_freq(uint32_t counter);
float set_FW_Speed(float speed);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Command_Parser();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
		Error_Handler();

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
		Error_Handler();

  if (HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1) != HAL_OK)
  		Error_Handler();

//while(1);
  UARTcmd_txBuffer[0] = sprintf((char*)&UARTcmd_txBuffer[1],"\r\n\nStarted !\r\n\n");
  HAL_UART_Transmit(&huart2, &UARTcmd_txBuffer[1], UARTcmd_txBuffer[0],10);

  UARTcmd_rxBuffer[0] = 1;
  if ( HAL_UART_Receive_IT(&huart2,&UARTcmd_rxBuffer[UARTcmd_rxBuffer[0]],1) != HAL_OK)
	  Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t to_IDLE_time=0;
  set_LED_Blink_freq(2000);
  _fw_target_speed = set_FW_Speed(fw_off_duty);

  while (1)
  {

	  switch(_fw_status)
	  {
	  case FW_STATUS_OFF:

		  switch (_fw_mode)
		  {
		  case FW_STATUS_OFF:
			  ; // No change
			  break;

		  case FW_STATUS_IDLE:
			  _fw_target_speed = set_FW_Speed(fw_idle_duty);
			  set_LED_Blink_freq(500);
			  to_IDLE_time = HAL_GetTick();
			  _fw_status = FW_STATUS_IDLE;
			  break;

		  case FW_STATUS_RUNNING:
			  _fw_mode = _fw_status; // Not valid transition
			  break;
		  }
		  break;

	  case FW_STATUS_IDLE:

		  switch (_fw_mode)
		  {
		  case FW_STATUS_OFF:
			  _fw_target_speed = set_FW_Speed(fw_off_duty);
			  _fw_status = FW_STATUS_OFF;
			  set_LED_Blink_freq(2000);
			  break;

		  case FW_STATUS_IDLE:
			  _fw_target_speed = set_FW_Speed(fw_idle_duty); // Keep target speed at idle
			  break;

		  case FW_STATUS_RUNNING:
			  if ( (to_IDLE_time + MIN_IDLE_TIME) < HAL_GetTick() )
			  {
				  _fw_target_speed = set_FW_Speed(fw_min_duty);
				  _fw_status = FW_STATUS_RUNNING;
				  set_LED_Blink_freq(10);
				  to_IDLE_time = HAL_GetTick();
			  }
			  break;
		  }
		  break;

	  case FW_STATUS_RUNNING:

		  switch (_fw_mode)
		  {
		  case FW_STATUS_OFF:
			  _fw_target_speed = set_FW_Speed(fw_off_duty);
			  _fw_status = FW_STATUS_OFF;
			  set_LED_Blink_freq(2000);
			  break;

		  case FW_STATUS_IDLE:
			  _fw_target_speed = set_FW_Speed(fw_idle_duty);
			  _fw_status = FW_STATUS_IDLE;
			  set_LED_Blink_freq(500);
			  break;

		  case FW_STATUS_RUNNING:
			  _fw_target_speed = (fw_min_duty < _fw_target_speed) ? _fw_target_speed : fw_min_duty;
			  _fw_target_speed = (_fw_target_speed < fw_max_duty) ? _fw_target_speed : fw_max_duty;

			  if ( (to_IDLE_time + MIN_STARTUP_TIME) < HAL_GetTick() )
			  {
				  _fw_target_speed = set_FW_Speed(_fw_target_speed);
			  }
			  break;
		  }
		  break;

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  __disable_irq();
	  UARTcmd_txBuffer[0] = sprintf((char*)&UARTcmd_txBuffer[1],"status:%lu mode:%lu rpm_tgt:%2lu error:%d\r\n", _fw_status, _fw_mode, (uint32_t) (_fw_target_speed*100.0),error);
//	  HAL_UART_Transmit(&huart2, &UARTcmd_txBuffer[1], UARTcmd_txBuffer[0],10);
	  __enable_irq();
//	  HAL_Delay(1000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 31999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 40000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
* @brief Blink the LED with half period and full period timer interrupt
* @param None
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		error = HAL_UART_Transmit(&huart2, &UARTcmd_txBuffer[1], UARTcmd_txBuffer[0],10);
	}

	if (htim->Instance == TIM7)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}
}
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7)
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
}

/**
* @brief Switch the FW mode between ready and idle
* @param counter in ms (Range 1 - 60000)
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		if (_fw_status == FW_STATUS_OFF)
			_fw_mode = FW_STATUS_IDLE;
		else
			_fw_mode = FW_STATUS_OFF;
	}
}

/**
* @brief Set the blinking period
* @param counter in ms (Range 1 - 60000)
* @retval None
*/
void set_LED_Blink_freq(uint32_t counter)
{
	if (0 < counter && counter < 65535)
	{
		uint32_t a = htim7.Instance->ARR;
		__HAL_TIM_SET_AUTORELOAD(&htim7, counter);
		__HAL_TIM_SET_COUNTER(&htim7, a);

	}
}

/**
* @brief Set PWM for FW control
* @param speed in % of range (Range 0 - 1)
* @retval None
*/
float set_FW_Speed(float speed)
{
	if (0 <= speed && speed <= 1)
		htim11.Instance->CCR1 = (uint32_t) ( speed * (float)htim11.Instance->ARR );

	return speed;
}

/**
* @brief Callback when data received on UART via DMA
* @param None
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{


		if (UARTcmd_rxBuffer[UARTcmd_rxBuffer[0]] == '\r') // We received an 'end of the line' char. Parsing command.
		{
			__disable_irq();
			UARTcmd_txBuffer[0] = sprintf((char*)&UARTcmd_txBuffer[1],"msg:%u:%c%c%c%c%c%c\n\r", UARTcmd_rxBuffer[0], UARTcmd_rxBuffer[1], UARTcmd_rxBuffer[2], UARTcmd_rxBuffer[3], UARTcmd_rxBuffer[4], UARTcmd_rxBuffer[5], UARTcmd_rxBuffer[6]);
			HAL_UART_Transmit(&huart2, &UARTcmd_txBuffer[1], UARTcmd_txBuffer[0],10);
			__enable_irq();

			Command_Parser();

			for (uint8_t i = 0; i < sizeof(UARTcmd_rxBuffer) ; i++) // Reseting the string
				UARTcmd_rxBuffer[i] = 0;
		}

		if (UARTcmd_rxBuffer[0] == sizeof(UARTcmd_rxBuffer)-1)
			UARTcmd_rxBuffer[0] = 0;

		UARTcmd_rxBuffer[0]++;

		error = HAL_UART_Receive_IT(&huart2, &UARTcmd_rxBuffer[UARTcmd_rxBuffer[0]], 1);

		if (error != HAL_OK)
			Error_Handler();
	}
}

/**
* @brief Parsing command received on UART
* @param None
* @retval None
*/
void Command_Parser()
{
	char option;
	unsigned int value;

	option = UARTcmd_rxBuffer[1];

	if (UARTcmd_rxBuffer[0] == 3) // one digit
	{
		if (48 <= UARTcmd_rxBuffer[2] && UARTcmd_rxBuffer[2] < 58)
			value = UARTcmd_rxBuffer[2]-48;
		else
			return;

	} else if (UARTcmd_rxBuffer[0] == 4) // two digit
	{
		if ( (48 <= UARTcmd_rxBuffer[2] && UARTcmd_rxBuffer[2] < 58)  &&
			 (48 <= UARTcmd_rxBuffer[3] && UARTcmd_rxBuffer[3] < 58) )

				value = UARTcmd_rxBuffer[3]-48 + 10*(UARTcmd_rxBuffer[2]-48);
		else
			return;
	} else
		return;


	switch(option)
	{
	case 'M':
	case 'm':
		if (0<=value && value <= 2)
			_fw_mode = value;
		break;

	case 'S':
	case 's':
		if (30<=value && value <= 99)
			_fw_target_speed = (float)value / 100.0;
		break;

	default:
		break;
	}
//	__disable_irq();
//	UARTcmd_txBuffer[0] = sprintf((char*)&UARTcmd_txBuffer[1],"mode:%c value:%u msg:%s\r\n", option, value, UARTcmd_rxBuffer);
//	HAL_UART_Transmit(&huart2, &UARTcmd_txBuffer[1], UARTcmd_txBuffer[0],10);
//	__enable_irq();
}
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
	HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	uint32_t i=500000;
	while(i--)
		asm("nop");
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
