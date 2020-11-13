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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//Different state of ATM machine
typedef enum
{
    RxNodos_State,
    Captura_Inhibicion_Tx_State,
    Espera_Estados_Rx_State,
    Reset_Tx_State
} eSystemState;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
  /* USER CODE BEGIN 2 */
  	eSystemState NextState;
	uint8_t RSSI_value[3];
	uint8_t estado_inhi[3];
	uint8_t cRx = 0;
	bool bPresencia_Inhibi;
	char in[3] = {0, 0, 0};
	char ch[3] = {0, 0, 0};


	NextState = RxNodos_State;
	bPresencia_Inhibi = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // DE - Comunicacion RS485 - Se coloca en bajo para estar en modo recepcion
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0); // RE - Comunicacion RS485 - Se coloca en bajo para escuchar todo el tiempoc
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Maquina de estados para comunicacion
	  switch(NextState){
		  case RxNodos_State:
			  in[0] = 0;
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 1000);

			  if(in[0] != 0){
				  bPresencia_Inhibi = 1;
				  NextState = Captura_Inhibicion_Tx_State;
	  	  	  }
			  else
				  NextState = RxNodos_State;

			  break;

		  case Captura_Inhibicion_Tx_State:
			  ch[0] = 'G';
			  // Pone en modo tx
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&ch, 1); //Definir palabra a enviar para que todos los nodos escuchen y guarden rssi y estado
			  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  NextState = Espera_Estados_Rx_State;

		  case Espera_Estados_Rx_State:

			  if 	  (cRx == 0) ch[0] = 'a';
			  else if (cRx == 1) ch[0] = 'b';
			  else if (cRx == 2) ch[0] = 'c';

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit_IT(&huart1, (uint8_t *)&ch, 1);
			  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  HAL_UART_Receive(&huart1, (uint8_t *)in, 3, 1000);
			  if(in[0] == 'A'){
				  RSSI_value[0] = in[1];
				  estado_inhi[0] = in[2];
				  cRx++;
			  }
			  else if(in[0] == 'B'){
				  RSSI_value[1] = in[1];
				  estado_inhi[1] = in[2];
				  cRx++;
			  }
			  else if(in[0] == 'C'){
				  RSSI_value[2] = in[1];
				  estado_inhi[2] = in[2];
				  cRx++;
			  }

			  if(cRx == 3){
				  cRx = 0;
				  NextState = Reset_Tx_State;
			  }
			  else
				  NextState = Espera_Estados_Rx_State;


			  break;
		  case Reset_Tx_State:
			  ch[0] = 'F';
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  bPresencia_Inhibi = 0;
			  NextState = RxNodos_State;

			  break;
		  default:
			  ch[0] = 'F';
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  bPresencia_Inhibi = 0;
			  NextState = RxNodos_State;
			  break;

	  };

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
  huart1.Init.BaudRate = 115200;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
