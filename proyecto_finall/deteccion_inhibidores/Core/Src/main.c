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
#include "rf_driver.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t agcctrl0;
uint8_t agcctrl1;
uint8_t agcctrl2;
uint16_t promedio, promediador, contador500ms = 0,  contador10cero_consecutivo = 0, contador200ms = 0;
uint8_t contador1000ms = 0;
_Bool banderadeuno, contador500ms_start, b10ms_clear = 0, bLed = 0;

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



  uint32_t error;
  uint8_t i = 2;
  uint16_t marcstate=0;


  //Init rf driver
  error = rf_begin(&hspi1, AKS_115_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
  rf_write_strobe(SRX);

  while(marcstate != RX){
  		marcstate = (rf_read_register(MARCSTATE)); //read out state of cc1100 to be sure in RX
  	}

  HAL_TIM_Base_Start_IT(&htim4);


  int result = rf_read_register(MARCSTATE);

//  test_cargar_cfg();

  while(i!=0){
	  i++;
//	  result = rf_read_register(0xF4);
//	  if(result < 128)  result = (int)(result/2)-74;
//	  else result = (int)((result -256)/2)-74;
	  if(i==250) i=1;
  }




  uint8_t boton;

  while (1)
     {
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) boton=1;
	  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) boton=2;
	  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) boton=3;
	  else boton=0;

	  switch(boton){
	  case 1:
		  agcctrl2r();
		  error = rf_begin(&hspi1, AKS_115_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
		  rf_set_carrier_frequency(433.92);
		  rf_set_carrier_offset(50);
		  test_cargar_cfg();
		  rf_write_register(0X1B, agcctrl2);
		  rf_write_strobe(SRX);
		  break;

	  case 2:
		  agcctrl1r();
		  error = rf_begin(&hspi1, AKS_115_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
		  rf_set_carrier_frequency(433.92);
		  rf_set_carrier_offset(50);
		  test_cargar_cfg();
		  rf_write_register(0X1C, agcctrl1);
		  rf_write_strobe(SRX);
		  break;

	  case 3:
		  agcctrl0r();
		  error = rf_begin(&hspi1, AKS_115_kb, MHz434, CS_GPIO_Port, CS_Pin, GDO0_Pin);
		  rf_set_carrier_frequency(433.92);
		  rf_set_carrier_offset(50);
		  test_cargar_cfg();
		  rf_write_register(0X1D, agcctrl0);
		  rf_write_strobe(SRX);
		  break;

	  default:
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(300);

	  }
     }


  /* USER CODE END 2 */

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
  huart1.Init.Parity = UART_PARITY_NONE;
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
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13 
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

void transmit_demo(uint16_t size){
	  uint8_t data[size];
	  int i;
	  uint8_t error = 0;

	  static int counter = 0;

	  data[0] = size & 0xFF;
	  data[1] = (size>>8) & 0xFF;

	  for(i=2; i<size; i++){
		  data[i] = (i)%256;
	  }

	  //while(1){
		  if(send_frame(data, size)!=FRAME_OK) error = 1;
		  data[size-1] = counter++;

	  //}
}

int test(){
return 0;
}



int agcctrl2r(){

	uint8_t  MAX_DVGA_GAIN, MAX_LNA_GAIN, MAGN_TARGET;
	uint8_t boton;
	agcctrl2 = rf_read_register(0x01B);
	HAL_Delay(100);

	while (1){

		MAX_DVGA_GAIN = (agcctrl2 & 0b11000000) >> 6;
		MAX_LNA_GAIN  = (agcctrl2 & 0b00111000) >> 3;
		MAGN_TARGET  = (agcctrl2 & 0b00000111);

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) boton=1;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) boton=2;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) boton=3;
		else boton=0;

		switch(boton){
		case 1:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl2 += 64; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl2 -= 64; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 2:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl2 += 8; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl2 -= 8; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 3:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl2 += 1; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl2 -= 1; return 0;}
			  HAL_Delay(300);
			}
			break;


		default:
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(300);

		}
	 }
}


int agcctrl1r(){

	uint8_t  AGC_LNA_PRIORITY, CARRIER_SENSE_REL_THR, CARRIER_SENSE_ABS_THR;

	agcctrl1 = rf_read_register(0x01C);
	HAL_Delay(100);

	uint8_t boton;


	while (1){

		AGC_LNA_PRIORITY = (agcctrl1 & 0b01000000) >> 6;
		CARRIER_SENSE_REL_THR  = (agcctrl1 & 0b00110000) >> 4;
		CARRIER_SENSE_ABS_THR  = (agcctrl1 & 0b00001111);

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) boton=1;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) boton=2;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) boton=3;
		else boton=0;

		switch(boton){
		case 1:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl1 += 64; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl1 -= 64; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 2:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl1 += 16; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl1 -= 16; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 3:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl1 += 1; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl1 -= 1; return 0;}
			  HAL_Delay(300);
			}
			break;


		default:
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(300);

		}
	 }


}

int agcctrl0r(){

	uint8_t  HYST_LEVEL, WAIT_TIME, AGC_FREEZE, FILTER_LENGTH;

	agcctrl0 = rf_read_register(0x01D);
	HAL_Delay(100);

	uint8_t boton;
	while (1){

		HYST_LEVEL = (agcctrl0 & 0b11000000) >> 6;
		WAIT_TIME  = (agcctrl0 & 0b00110000) >> 4;
		AGC_FREEZE  = (agcctrl0 & 0b00001100) >> 2;
		FILTER_LENGTH = (agcctrl0 & 0b00000011);

		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) boton=1;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) boton=2;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) boton=3;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) boton=4;
		else boton=0;

		switch(boton){
		case 1:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl0 += 64; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl0 -= 64; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 2:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl0 += 16; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl0 -= 16; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 3:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl0 += 4; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl0 -= 4; return 0;}
			  HAL_Delay(300);
			}
			break;

		case 4:
			while(1){
			  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {agcctrl0 += 1; return 0;}
			  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)) {agcctrl0 -= 1; return 0;}
			  HAL_Delay(300);
			}
			break;


		default:
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		  HAL_Delay(300);

		}
	 }


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	int result_RSSI;
	uint8_t result;


	if(htim->Instance == TIM4){
		contador200ms++;

		if(contador200ms == 4999){
			contador200ms = 0;

			result = rf_read_register(RSSI);
			if(result < 128)  result_RSSI = (int)(result/2)-74;
			else result_RSSI = (int)((result -256)/2)-74;

			RSSI_level(result_RSSI);
		}

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1) contador500ms_start = 1;

		if(contador500ms_start == 1) {
			contador500ms++;

			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) {
				if(banderadeuno == 0) contador10cero_consecutivo++;
				banderadeuno=0;

			}

			else {
				promediador++;
				if(banderadeuno == 1)contador10cero_consecutivo = 0;
				banderadeuno = 1;
				b10ms_clear=0;
			}

		}
		if(contador500ms == 12500) {

			if(contador10cero_consecutivo >= 250 ){
				b10ms_clear=1;
			}
			promedio = (int)((promediador*100)/12500);
			if (promedio > 40){
				bLed = 1;
			}
			contador500ms = 0;
			contador500ms_start = 0;
			contador10cero_consecutivo = 0;
			promediador = 0;
			contador1000ms++;


			if(contador1000ms < 2){
				if(bLed == 1)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
			}
			else {
				if(bLed == 0)HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
				bLed = 0;
				contador1000ms = 0;
			}


		}
	}

}

void RSSI_level(int RSSI_lvl){

	int ledLevel;
    uint16_t puerto = 1024;

	ledLevel = (0.06*RSSI_lvl+7.2);

	for(int i=0; i<7; i++){
		if(i!=0) puerto = puerto * i;

		if(i < ledLevel){
			HAL_GPIO_WritePin(GPIOB, puerto, 1);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, puerto, 0);
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
