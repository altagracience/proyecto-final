/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t contador1s = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//Different state of ATM machine
typedef enum
{
    No_Inhibicion_State,
    Inhibicion_State,
    Reset_State
} eSystemState;

TIM_HandleTypeDef htim4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

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
//  HAL_UART_IRQHandler(&huart1);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	eSystemState NextState;
	uint8_t RSSI_value[3] = {0,0,0};
	uint8_t estado_inhi[3] = {0,0,0};
	uint8_t cRx = 0;
	uint8_t err = 0;
	uint8_t i = 0;
	char in[3] = {0, 0, 0};
	char ch[2] = {0, 0};


	NextState = No_Inhibicion_State;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // DE - Comunicacion RS485 - Se coloca en bajo para estar en modo recepcion
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0); // RE - Comunicacion RS485 - Se coloca en bajo para escuchar todo el tiempoc


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  switch(NextState){
	  	  case No_Inhibicion_State:

	  		  HAL_Delay(200);

	  		  ch[1] = 0;
	  		  in[2] = 0;

			  if 	  (cRx == 0) ch[0] = 'A';
			  else if (cRx == 1) ch[0] = 'B';
			  else if (cRx == 2) ch[0] = 'C';


	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	  		  HAL_Delay(50);
	  		  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 2, 50);
	  		  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 10);
	  		  HAL_Delay(50);
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

	  		  in[2] = in[1] = in[0] = 0;
	  		  err = HAL_UART_Receive(&huart1, (uint8_t *)in, 3, 100);

	  		  if(err==3){

	  		  }

	  		  if (cRx >= 2)
	  			  cRx = 0;

	  		  else cRx++;

	  		  if(in[2] == 1 || in[2] == 2){
	  			  NextState = Inhibicion_State;
	  			  cRx = 0;
	  		  }

	  		  break;

	  	  case Inhibicion_State:

	  		  ch[1] = 1;
	  		  in[0] = in[1] = in[2] = 0;


	  		  if 	  (cRx == 0) ch[0] = 'A';
	  		  else if (cRx == 1) ch[0] = 'B';
	  		  else if (cRx == 2) ch[0] = 'C';

	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	  		HAL_Delay(50);
	  		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 2, 50);
	  		HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 10);
	  		HAL_Delay(50);
	  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  in[2] = in[1] = in[0] = 0;

			  err = HAL_UART_Receive(&huart1, (uint8_t *)in, 3, 10);

			  if(in[0] == 'A'){
				  if (in[1] > 120 || in[1] < 18) RSSI_value[0] = 0;
				  else RSSI_value[0] = in[1];

				  if (in[2] > 2) estado_inhi[0] = 0;
				  else estado_inhi[0] = in[2];

			  }
			  else if(in[0] == 'B'){
				  if (in[1] > 120 || in[1] < 18) RSSI_value[1] = 0;
				  else RSSI_value[1] = in[1];

				  if (in[2] > 2) estado_inhi[1] = 0;
				  else estado_inhi[1] = in[2];
			  }
			  else if(in[0] == 'C'){
				  if (in[1] > 120 || in[1] < 18) RSSI_value[2] = 0;
				  else RSSI_value[2] = in[1];

				  if (in[2] > 2) estado_inhi[2] = 0;
				  else estado_inhi[2] = in[2];
			  }

			  cRx++;

			  if (cRx == 3){
				  if(estado_inhi[0] || estado_inhi[1] || estado_inhi[2]) //desata alarma visual y sonora en la central
					  HAL_TIM_Base_Start_IT(&htim4);

				  NextState = Reset_State;


			  }
			  break;

	  	  case Reset_State:

	  		  for(i = 0; i < 2; i++){

	  			  ch[0] = 'A';
				  ch[1] = 2;

				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
				  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 2, 50);
				  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 10);
				  HAL_Delay(50);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	  		  }

	  		  estado_inhi[0] = estado_inhi[1] = estado_inhi[2] = 0;
	  		  RSSI_value[0] = RSSI_value[1] = RSSI_value[2] = 0;

			  NextState = No_Inhibicion_State;
			  break;

	  	  default:
	  		NextState = Reset_State;


	  }




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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1439;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


	if(htim->Instance == TIM4){ //chequea que la interrupción sea la del timer adecuado

		contador1s++;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		if(contador1s == 5000) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
			contador1s = 0;
			HAL_TIM_Base_Stop_IT(&htim4);
		}

	}

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
