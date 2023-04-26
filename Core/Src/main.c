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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050.h"
#include "FIRFilter.h"
#include "RCFilter.h"
#include "FirstOrderIIR.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_TIME_LOG_MS 50 	/* 50ms ==> 20Hz */
#define SAMPLE_TIME_LED_MS 500 		/* 500ms ==> 2Hz */
#define LOOP_TIME_MS 1 /* 1000us == 1ms => 1kHz */
#define ALPHA 0.15f /* Alpha value for the complementary filter */
#define ACC_FILTER_ALPHA 0.10f /* Alpha value for accelerometer IIR filter */
#define GYRO_FILTER_ALPHA 0.01f /* Alpha value for gyro IIR filter */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
/* Create an IMU instance */
MPU6050 IMU;

/* Create FIR filter instances */
FIRFilter MAFilter[2];

/* Create a first order IIR filter instance */
 FirstOrderIIR lpfAcc[3];
 FirstOrderIIR lpfGyro[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	uint8_t CDC_Transmit_FS(uint8_t*, int);

	/* Initialize IMU */
	MPU6050_Initialise(&IMU, &hi2c1);

	/* Initializes Moving Average filter (10 point) */
	FIRFilter_Init(&MAFilter[0]);

	FIRFilter_Init(&MAFilter[1]);

	/* Initialize First order IIR filter */
	for (int n = 0; n < 3; n++) {

		FirstOrderIIR_Init(&lpfAcc[n], ACC_FILTER_ALPHA);

		FirstOrderIIR_Init(&lpfGyro[n], GYRO_FILTER_ALPHA);

	}



	/* USB data buffer */
	char usbBuf[128];

	/* Timer variables */
	uint32_t timerLog = 0;
	uint32_t timerLed = 0;
	uint32_t timerLoop = 0;

	/* Estimated angles from the accelerometer */
	float phi_est_acc = 0;
	float theta_est_acc = 0;
	float phi_est_gyro = 0;
	float theta_est_gyro = 0;
	float theta_est = 0;
	float phi_est = 0;
	float phi_dot_eul = 0;
	float theta_dot_eul = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* IMU update loop */
		if ((HAL_GetTick() - timerLoop) >= LOOP_TIME_MS) {

			/* Read Orientation */
			MPU6050_ReadOrientation(&IMU);

			/* Read temperature */
			MPU6050_ReadTemperature(&IMU);

			/* Read Accelerations */
			MPU6050_ReadAccelerations(&IMU);

			/* Fitering accelerometer data (test) */

			for (int n = 0; n < 3; n++) {

				FirstOrderIIR_Update(&lpfAcc[n], IMU.acc_mps2[n]);

				FirstOrderIIR_Update(&lpfGyro[n], IMU.gyro_deg[n]);

			}

			/* Getting theta and phi estimates from the accelerometer */
			phi_est_acc = atan2f(lpfAcc[1].out, lpfAcc[2].out);
			theta_est_acc = asinf(lpfAcc[0].out / 9.81);

			/* Getting theta and phi estimates from the gyroscope (by transforming the body rates to euler rates) */
			phi_dot_eul = lpfGyro[0].out
					+ tanf(theta_est)
							* (sinf(phi_est) * lpfGyro[1].out
									+ cosf(phi_est) * lpfGyro[2].out);

			theta_dot_eul = cosf(phi_est) * lpfGyro[1].out
					- sinf(phi_est) * lpfGyro[2].out;

			/* Integrating gyro values to get theta and phi estimates */
			theta_est_gyro = (SAMPLE_TIME_LOG_MS / 1000.0f) * theta_dot_eul;

			phi_est_gyro = (SAMPLE_TIME_LOG_MS / 1000.0f) * phi_dot_eul;

			/* Combining accelerometer estimates with integral of gyro readings */
			phi_est = ALPHA * phi_est_acc
					+ (1.0f - ALPHA) * (phi_est + phi_est_gyro);
			theta_est = ALPHA * theta_est_acc
					+ (1.0f - ALPHA) * (theta_est + theta_est_gyro);

//
//

			timerLoop += LOOP_TIME_MS;

		}

		/* Send data via virtual COM port */
		if ((HAL_GetTick() - timerLog) >= SAMPLE_TIME_LOG_MS) {

			/* printing out theta and phi estimates */
//			uint8_t usbBufLen = snprintf(usbBuf, 128, "%.3f, %.3f \r\n",
//					IMU.gyro_deg[2], lpfGyroRC[2].out[0]);

			/* Smoothen phi and theta estimates */
//			FIRFilter_Update(&MAFilter[0], theta_est);
//
//			FIRFilter_Update(&MAFilter[1], phi_est);

			/* printing out theta and phi estimates */
			uint8_t usbBufLen = snprintf(usbBuf, 128, "%.3f, %.3f \r\n",
					theta_est * RAD_TO_DEG, phi_est * RAD_TO_DEG);

//			uint8_t usbBufLen = snprintf(usbBuf, 128,
//					"%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\r\n",IMU.gyro_deg[0], IMU.gyro_deg[1], IMU.gyro_deg[2], IMU.acc_mps2[0],
//					IMU.acc_mps2[1], IMU.acc_mps2[2], IMU.temp_C);

			CDC_Transmit_FS((uint8_t*) usbBuf, usbBufLen);

			timerLog += SAMPLE_TIME_LOG_MS;

		}

		/* Toggle LED */
		if ((HAL_GetTick() - timerLed) >= SAMPLE_TIME_LED_MS) {

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			timerLed += SAMPLE_TIME_LED_MS;

		}

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
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
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
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
