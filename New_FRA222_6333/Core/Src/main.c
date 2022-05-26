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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0b10100000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t Temp = 0;
uint16_t TempOld[100] = { 0 };
uint8_t TempNow = 0;

char TxDataBuffer[32] = { 0 };
char RxDataBuffer[3] = { 0 };

uint8_t time_wait = 1;
uint8_t time_work = 1;
uint8_t station_goal = 1;

uint8_t eepromWriteFlag = 0;
uint8_t eepromReadFlag = 0;
uint8_t eepromDataReadBack[3];

uint8_t IOExpdrWrite = 0b01000000;
uint8_t IOExpdrRead = 0b01000001;
uint8_t for_spi[2] = { 0 };

uint64_t _micro = 0;

char H0[] =
		"linku starto.type something to start\r\n howto use: w is +1, s is -1 if finisf press e\r\n";
char H1[] = "select station \r\n";
char H2[] = "waiting time from station to station\r\n";
char H3[] = "working time when robot reach the station\r\n";
char H4[] = "run the program. press e to setting again.\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void UI_UART();

uint64_t micros();

void EEPROMWrite();
void EEPROMRead(uint8_t *Rdata, uint16_t len);

void IOExpenderInit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_SPI3_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_UART_Transmit(&huart2, (uint8_t*) H0, strlen(H0), 1000);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &Temp, 1);
	IOExpenderInit();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		EEPROMWrite();
		EEPROMRead(eepromDataReadBack, 3);
		HAL_UART_Receive_IT(&huart2, RxDataBuffer, 1);

		TempNow = (((Temp*3.6/256)-0.76 )/2.5)+25;

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 99;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 65535;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_SS_Pin */
	GPIO_InitStruct.Pin = SPI_SS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void UI_UART() {
	static uint8_t n_state = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*) H1, strlen(H1), 10);
	switch (n_state) {
	case 0:
		switch (RxDataBuffer[0]) {
		case 'w':
			station_goal += 1;
			sprintf(TxDataBuffer, "station_goal:[%d]\r\n", station_goal);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 's':
			station_goal -= 1;
			sprintf(TxDataBuffer, "station_goal:[%d]\r\n", station_goal);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 'e':
			HAL_UART_Transmit(&huart2, (uint8_t*) H2, strlen(H2), 10);
			n_state = 1;
			break;
		}
		break;
	case 1:
		switch (RxDataBuffer[0]) {
		case 'w':
			time_wait += 1;
			sprintf(TxDataBuffer, "time_wait:[%d]\r\n", time_wait);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 's':
			time_wait -= 1;
			sprintf(TxDataBuffer, "time_wait:[%d]\r\n", time_wait);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 'e':
			HAL_UART_Transmit(&huart2, (uint8_t*) H3, strlen(H3), 10);
			n_state = 2;
			break;
		}
		break;
	case 2:
		switch (RxDataBuffer[0]) {
		case 'w':
			time_work += 1;
			sprintf(TxDataBuffer, "time_work:[%d]\r\n", time_work);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 's':
			time_work -= 1;
			sprintf(TxDataBuffer, "time_work:[%d]\r\n", time_work);
			HAL_UART_Transmit(&huart2, (uint8_t*) TxDataBuffer,
					strlen(TxDataBuffer), 1000);
			RxDataBuffer[0] = 0;
			break;
		case 'e':
			HAL_UART_Transmit(&huart2, (uint8_t*) H4, strlen(H4), 10);
			eepromWriteFlag = 1;
			n_state = 3;
			break;
		}
		break;
	case 3:
		/*static uint64_t timestamp = 0;
		 static uint8_t state_LED = 1;
		 if (micros() - time_wait > time_wait && state_LED = 1) {
		 timestamp = micros();
		 if (hspi3.State == HAL_SPI_STATE_READY
		 && HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
		 == GPIO_PIN_SET) {
		 MCP4922SetOutput(DACConfig, dataOut);
		 }
		 }
		 else if(micros() - time_wait > time_wait){

		 }*/
		switch (RxDataBuffer[0]) {

		case 'e':
			n_state = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	UI_UART();
}

void EEPROMWrite() {
	if (eepromWriteFlag && hi2c1.State == HAL_I2C_STATE_READY) {

		static uint8_t data[3] = { 0 };
		data[0] = station_goal;
		data[1] = time_wait;
		data[2] = time_work;

		HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_ADDR, 0x1A, I2C_MEMADD_SIZE_16BIT,
				data, 3);

		eepromWriteFlag = 0;
	}
}

void EEPROMRead(uint8_t *Rdata, uint16_t len) {
	if (eepromReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {

		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x1a, I2C_MEMADD_SIZE_16BIT,
				Rdata, len);

		eepromReadFlag = 0;
	}
}

void IOExpenderInit() {
	//Init All
	/*static uint8_t Setting[0x16] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };
	for_spi[0] = 0b01000000;
	 for_spi[1] = Setting;
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, Setting, 16);*/
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi3) {
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}
/*void IOExpenderReadPinA(uint8_t *Rdata) {
 if (HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET
 && hspi3.State == HAL_SPI_STATE_READY) {
 HAL_SPI_Mem_Read_IT(&hspi3, IOEXPD_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT,
 Rdata, 1);
 IOExpdrReadFlag = 0;
 }
 }
 void IOExpenderWritePinB(uint8_t Wdata) {
 if (HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin) == GPIO_PIN_SET
 && hspi3.State == HAL_SPI_STATE_READY) {
 static uint8_t data;
 data = Wdata;
 HAL_SPI_Mem_Write_IT(&hspi3, IOEXPD_ADDR, 0x15, I2C_MEMADD_SIZE_8BIT,
 &data, 1);
 IOExpdreWriteFlag = 0;
 }
*/
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 if (htim == &htim11) {
 _micro += 65535;
 }
 }

 inline uint64_t micros() {
 return htim11.Instance->CNT + _micro;
 }
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

