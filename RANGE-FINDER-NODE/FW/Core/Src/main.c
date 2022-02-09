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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "vl53l0x_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L0_ADDR 0x52
#define VL53L0_I2CH hi2c1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t vl53l0x_c; // center module
VL53L0X_DEV Dev = &vl53l0x_c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

typedef enum {
	LONG_RANGE = 0, /*!< Long range mode */
	HIGH_SPEED = 1, /*!< High speed mode */
	HIGH_ACCURACY = 2, /*!< High accuracy mode */
} RangingConfig_e;

VL53L0X_RangingMeasurementData_t RangingMeasurementData;
//VL53L0X_Dev_t VL53L0XDevs[]={
//        {.Id=XNUCLEO53L0A1_DEV_LEFT, .DevLetter='l', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
//        {.Id=XNUCLEO53L0A1_DEV_CENTER, .DevLetter='c', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
//        {.Id=XNUCLEO53L0A1_DEV_RIGHT, .DevLetter='r', .I2cHandle=&XNUCLEO53L0A1_hi2c, .I2cDevAddr=0x52},
//};

VL53L0X_Dev_t VL53L0XDevs[] = { { .I2cHandle = &hi2c1 } };
int LeakyFactorFix8 = (int) (0.6 * 256);
int nDevPresent = 0;

int sensor_status;
int sensors = 0;

RangingConfig_e RangingConfig;

/**
 * Reset all sensor then do presence detection
 *
 * All present devices are data initiated and assigned to their final I2C address
 * @return
 */
int DetectSensors(uint8_t addr) {
	int i = 0;
	uint16_t Id;
	volatile int status = 0;
	//   int FinalAddress;

//    for (i = 0; i < 3; i++) {
	VL53L0X_Dev_t *pDev;
	pDev = &VL53L0XDevs[i];
	pDev->I2cHandle = &hi2c1;
	pDev->I2cDevAddr = addr;
	pDev->Present = 0;
	HAL_Delay(2);
//	FinalAddress=addr+(i+1)*2;

	do {
		/* Set I2C standard mode (400 KHz) before doing the first register access */
		if (status == VL53L0X_ERROR_NONE)
			status = VL53L0X_WrByte(pDev, 0x88, 0x00);

		/* Try to read one register using default 0x52 address */
		status = VL53L0X_RdWord(pDev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if (status) {
			break;
		}
		if (Id == 0xEEAA) {
//			/* Sensor is found */
			nDevPresent++;
			pDev->Present = 1;
		} else {
			status = 1;
		}
	} while (0);

	return nDevPresent;
}

/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
void SetupSingleShot(RangingConfig_e rangingConfig) {
	int i = 0;
	int status;
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t) (0.25 * 65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t) (18 * 65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	if (VL53L0XDevs[i].Present) {
		status = VL53L0X_StaticInit(&VL53L0XDevs[i]);

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings,
				&PhaseCal);

		status = VL53L0X_PerformRefSpadManagement(&VL53L0XDevs[i],
				&refSpadCount, &isApertureSpads);

		status = VL53L0X_SetDeviceMode(&VL53L0XDevs[i],
				VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i],
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit

		status = VL53L0X_SetLimitCheckEnable(&VL53L0XDevs[i],
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit

		/* Ranging configuration */
		switch (rangingConfig) {
		case LONG_RANGE:
			signalLimit = (FixPoint1616_t) (0.1 * 65536);
			sigmaLimit = (FixPoint1616_t) (60 * 65536);
			timingBudget = 33000;
			preRangeVcselPeriod = 18;
			finalRangeVcselPeriod = 14;
			break;
		case HIGH_ACCURACY:
			signalLimit = (FixPoint1616_t) (0.25 * 65536);
			sigmaLimit = (FixPoint1616_t) (18 * 65536);
			timingBudget = 200000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
		case HIGH_SPEED:
			signalLimit = (FixPoint1616_t) (0.25 * 65536);
			sigmaLimit = (FixPoint1616_t) (32 * 65536);
			timingBudget = 20000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
		default:
			break;
		}

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);

		status = VL53L0X_SetLimitCheckValue(&VL53L0XDevs[i],
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);

		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&VL53L0XDevs[i],
				timingBudget);

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],
				VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);

		status = VL53L0X_SetVcselPulsePeriod(&VL53L0XDevs[i],
				VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);

		status = VL53L0X_PerformRefCalibration(&VL53L0XDevs[i], &VhvSettings,
				&PhaseCal);

		VL53L0XDevs[i].LeakyFirst = 1;

	}
}

/* Store new ranging data into the device structure, apply leaky integrator if needed */
void Sensor_SetNewRange(VL53L0X_Dev_t *pDev,
		VL53L0X_RangingMeasurementData_t *pRange) {
	if (pRange->RangeStatus == 0) {
		if (pDev->LeakyFirst) {
			pDev->LeakyFirst = 0;
			pDev->LeakyRange = pRange->RangeMilliMeter;
		} else {
			pDev->LeakyRange = (pDev->LeakyRange * LeakyFactorFix8
					+ (256 - LeakyFactorFix8) * pRange->RangeMilliMeter) >> 8;
		}
	} else {
		pDev->LeakyFirst = 1;
	}
}

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
	char msg[16];
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
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	// LCD  init
	lcd_init();

	// VL53L0X init for Single Measurement

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	lcd_send_string("RacoonLab IU");
	HAL_Delay(100);
	// getting address
	for (uint16_t i = 0; i < 255; i++) {
		nDevPresent = DetectSensors(i);
		if (nDevPresent != 0) {
			if (i == 0x4E) //if the detected address is the LCD node.
					{
				continue;
			}
			lcd_clear();
			lcd_put_cur(0, 0);
			sprintf(msg, "SEN_ADDR=0X%X", i);
			lcd_send_string(msg);
			VL53L0XDevs[0].I2cDevAddr = i;
			lcd_put_cur(1, 0);
			lcd_send_string("ADDRESS SET");
			HAL_Delay(50);
			break;
		}

	}
	RangingConfig = HIGH_ACCURACY;
	HAL_Delay(1000);
	/* Set VL53L0X API trace level */
	VL53L0X_trace_config(NULL, TRACE_MODULE_NONE, TRACE_LEVEL_NONE,
			TRACE_FUNCTION_NONE); // No Trace
	//VL53L0X_trace_config(NULL,TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL); // Full trace
	sensor_status = VL53L0X_DataInit(&VL53L0XDevs[0]);
	sensor_status = VL53L0X_StaticInit(&VL53L0XDevs[0]);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		SetupSingleShot(RangingConfig);
		/* only one sensor */
		/* Call All-In-One blocking API function */
		sensor_status = VL53L0X_PerformSingleRangingMeasurement(&VL53L0XDevs[0],
				&RangingMeasurementData);
		HAL_Delay(100);
		RangingConfig = HIGH_ACCURACY;
		lcd_clear();
		lcd_put_cur(0, 0);
		lcd_send_string("MEASURE REPORT");
		lcd_put_cur(1, 0);
		if (RangingMeasurementData.RangeStatus == 0) {
			sprintf(msg, "D=%dmm :D", RangingMeasurementData.RangeMilliMeter);
			lcd_send_string(msg);
		} else {
			sprintf(msg,"ERR CODE %d (O_o)",RangingMeasurementData.RangeStatus);
			lcd_send_string(msg);

		}
		sensor_status = VL53L0X_ResetDevice(&VL53L0XDevs[0]);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
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
	hi2c1.Init.ClockSpeed = 100000;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

