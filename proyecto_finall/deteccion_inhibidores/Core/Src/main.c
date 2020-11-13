/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Cuerpo de programa para detección de inhibiciones
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
#include "rf_driver.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ID_nodo  'C'

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint16_t promedio = 0, a_promediar = 0, contador20ms = 0, contador150ms = 0, contador20ms_rssi = 0;
uint16_t sync_mx = 1; 			//variable de control para sincronización máxima de 320ms
uint8_t state = 0, i_rssi=0;
uint16_t suma_RSSI = 90;
uint8_t RSSI_val[10] = {90,90,90,90,90,90,90,90,90,90};


//Different state of ATM machine
typedef enum
{
    RxJammer_State,
    Info_central_Tx_State,
	Guarda_RSSI_Rx_State,
    Espera_Rx_State,
    Envia_Tx_State,
    Reset_Rx_State
} eSystemState;

uint8_t bInhibicion;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  	eSystemState NextState;
  	uint8_t gRSSI_value;

  	char ch[3];
  	char in[3]={0, 0, 0};
  	uint8_t gInhibicion;
  	uint16_t err;


  	bInhibicion = 0;
  	NextState = RxJammer_State;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // DE - Comunicacion RS485 - Se coloca en bajo para estar en modo recepcion
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0); // RE - Comunicacion RS485 - Se coloca en bajo para escuchar todo el tiempo

	uint16_t marcstate=0;


	//Init rf driver
	rf_begin(&hspi1, AKS_115_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
	rf_write_strobe(SRX);

	while(marcstate != RX){
		marcstate = (rf_read_register(MARCSTATE)); //read out state of cc1100 to be sure in RX
	}

	HAL_TIM_Base_Start_IT(&htim4);



	//  test_cargar_cfg();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1){
			HAL_TIM_Base_Start_IT(&htim4);
		  }*/

		// Maquina de estados para comunicacion
		switch(NextState){
		  case RxJammer_State:
			  // Codigo que corre el estado
			  in[0] = in[1] = in[2] = 0;
			  HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 1000);
			  if(in[0] != 0)
				  NextState = Espera_Rx_State;
			  else if(bInhibicion != 0)
				  NextState = Info_central_Tx_State;
			  else
				  NextState = RxJammer_State;
			  break;
		  case Info_central_Tx_State:
			  ch[0] = 'I'; //Palabra a transmitir cuando detecta inhibicion
			  HAL_TIM_Base_Stop_IT(&htim4);

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10); //Definir palabra a enviar
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			  NextState = Espera_Rx_State;

			  break;
		  case Guarda_RSSI_Rx_State:

			  gRSSI_value = RSSI_level();
			  gInhibicion = bInhibicion;
			  NextState = Espera_Rx_State;

			  break;
		  case Espera_Rx_State:
			  HAL_TIM_Base_Stop_IT(&htim4);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
			  err = HAL_UART_Receive(&huart1, (uint8_t *)in, 1, 12000);


			  if(in[0] == 'G')
				  NextState = Guarda_RSSI_Rx_State;
			  else if(in[0] == (ID_nodo + 32))
				  NextState = Envia_Tx_State;
			  else if(in[0] == 'F') //En el char 1 se pone F de fin
				  NextState = Reset_Rx_State;
			  else
				  NextState = Espera_Rx_State;

			  if(err ==  3) NextState = Reset_Rx_State;

			  break;
		  case Envia_Tx_State:
			  ch[0] = ID_nodo;
			  ch[1] = gRSSI_value;
			  ch[2] = gInhibicion;

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
			  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 3, 10); //Definir palabra a enviar, deberia enviar ID RSSI MODINHIBICION
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);

			  NextState = Espera_Rx_State;

			  break;
		  case Reset_Rx_State:
			  HAL_TIM_Base_Start_IT(&htim4);
			  bInhibicion = 0;
			  NextState = RxJammer_State;
			  in[0] = in[1] = in[2] = 0;
			  break;
		  default:
			  HAL_TIM_Base_Start_IT(&htim4);
			  bInhibicion = 0;
			  NextState = RxJammer_State;
			  //Resetear sistema ?
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim4.Init.Period = 1;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GDO0_Pin */
  GPIO_InitStruct.Pin = GDO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GDO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin PA11 */
  GPIO_InitStruct.Pin = CS_Pin|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB15 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_15|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	uint8_t data_in;


	if(htim->Instance == TIM4){ //chequea que la interrupción sea la del timer adecuado


		data_in = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2); //leo el valor de dato de entrada


		switch(state)
		{
			case 0:
				a_promediar += data_in;
				contador20ms++;
				if(contador20ms == 499){ 							// 500 cuentas equivalen a 20ms por la frecuencia de 25kHz de la interrupcion
					promedio = (int) ((a_promediar * 100) / 500); 	//calculo el promedio de 1 en 20 ms

					if(promedio > threshold) sync_mx = sync_mx << 1; //sync_mx es una variable de 16 bits que me permite testear cuantos bloques consecutivos de 20ms hubo un porcentaje de 1 que supere el threshold
					else sync_mx = 1;

 					if(sync_mx == 0)  {
						state = 1; //si surgen 16 bloques consecutivos sospechamos inhibición y hacemos análisis en 150ms
						sync_mx = 1;
					}
					contador20ms = 0;
					a_promediar = 0;
					bInhibicion = 0;
					promedio = 0;
				}
				break;

			case 1:
				a_promediar += data_in;
				contador150ms++;
				if(contador150ms == 3749) {
					promedio = (int) ((a_promediar * 100) / 3750); 	//calculo el promedio de unos en 150 ms
					if(promedio > threshold){
						bInhibicion = 1;
						HAL_TIM_Base_Stop_IT(&htim4);}
					contador150ms = 0; // 3750 cuentas equivalen a 150ms por la frecuencia de 25kHz de la interrupcion
					state = 0;
					a_promediar = 0;
					promedio = 0;
				}

				break;
		}

		contador20ms_rssi++;
		if(contador20ms_rssi == 499){ 							// 500 cuentas equivalen a 20ms por la frecuencia de 25kHz de la interrupcion

			if(i_rssi < 10){
				RSSI_val[i_rssi] = RSSI_level(); 					//Llamo a función que devuelve el valor del rssi
				if (RSSI_val[i_rssi] > 120 || RSSI_val[i_rssi] < 18) RSSI_val[i_rssi] = suma_RSSI;
				i_rssi++;
			}
			else i_rssi = 0;


			for(int i = 0; i<10; i++){
				suma_RSSI += RSSI_val[i];
			}

			suma_RSSI = (uint16_t)(suma_RSSI/10);

			if(suma_RSSI <= 30) {
				bInhibicion = 2;
				HAL_TIM_Base_Stop_IT(&htim4);  // Si RSSI promedio es <= 30 es inhibicion por potencia
			}
			contador20ms_rssi = 0;


		}
	}

}

uint8_t RSSI_level(){ //esta funcion devuelve el valor en dBm del RSSI
	int result_RSSI, result;

	result = rf_read_register(RSSI); 			//leo el registro que almacena el RSSI
	if(result < 128){  							//Transformo a dBm segun su valor
		result_RSSI = (int)(result/2)-74;
	}
	else{
		result_RSSI = (int)((result -256)/2)-74;
	}


	return (uint8_t)((-1)*result_RSSI);
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
