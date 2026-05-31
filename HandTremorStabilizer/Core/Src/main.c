/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "errno.h"
#include <sys/unistd.h>
#include <string.h>
#include <stdio.h>

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float gyro_x_dps;
float gyro_y_dps;
float gyro_z_dps;

float accel_x_raw;
float accel_y_raw;
float accel_z_raw;

// accel + complementary filter variables
float fused_pitch = 0.0f;
float fused_roll  = 0.0f;
float fused_yaw   = 0.0f;


float alpha = 0.98f;
float dt = 0.02f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

int8_t current_step = 0;
void set_servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
    if (angle > 180) angle = 180;

    // 50 is 1ms (0 degrees), 100 is 2ms (180 degrees)
    uint32_t pulse_length = 50 + (angle * 50 / 180);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}

void Stepper_SetStep(uint8_t step) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, (step == 0 || step == 1 || step == 7));
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, (step == 1 || step == 2 || step == 3));
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, (step == 3 || step == 4 || step == 5));
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, (step == 5 || step == 6 || step == 7));
}

void Move_Stepper(int8_t direction) {
    // direction: 1 for forward (pull), -1 for backward (weaken)
    current_step += direction;

    // Keep current_step within 0-7 range
    if (current_step > 7) current_step = 0;
    if (current_step < 0) current_step = 7;

    Stepper_SetStep(current_step);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
				{
				  // Check if we are writing to stdout or stderr
				  if ((file == STDOUT_FILENO) || (file == STDERR_FILENO))
				  {
					HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
					if (status == HAL_OK) return len;
					else return EIO; // Input/Output error
				  }
				  errno = EBADF; // Bad file number
				  return -1;
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
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t whoAmI = 0;
  uint8_t gyro_cfg = 0x46;
  uint8_t accel_cfg = 0x4C;

  uint8_t ctrl1_addr = 0x10;
  uint8_t ctrl2_addr = 0x11;

  uint8_t ctrl4_addr = 0x13; // LPF1 and Bandwidth
  uint8_t ctrl7_addr = 0x16; // High-pass filter
  uint8_t ctrl9_addr = 0x18; // Denoising (DEN) configuration

  // Filter Configuration Values
  uint8_t lpf_denoise_cfg = 0x02; // LPF1_SEL_G = 1
  uint8_t hpf_cfg         = 0x40; // HP_EN_G = 1 (removes drift)

  uint8_t enable_axes = 0x38; // Enable X, Y, Z axes

  uint8_t ctrl3_auto_inc = 0x04; // Set IF_INC bit


  // 1. Accelerometer config
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, ctrl1_addr, 1, &accel_cfg, 1, 100);
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, 0x1E, 1, &enable_axes, 1, 100); // Register 0x1E is often FUNC_CFG_EN
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, 0x12, 1, &ctrl3_auto_inc, 1, 100);
   	  	  	  	  	  	  	  	  	  	  	    //   | one byte long
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, ctrl2_addr, 1, &gyro_cfg, 1, 100); //power + configuration of gyro
   // 2. Enable Low-Pass Filter (Denoising step 1)
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, ctrl4_addr, 1, &lpf_denoise_cfg, 1, 100);
   // 3. Enable High-Pass Filter (Removes slow "creeping" drift)
   HAL_I2C_Mem_Write(&hi2c2, 0xD4, ctrl7_addr, 1, &hpf_cfg, 1, 100);

   uint8_t buffer[6]; // Array to hold the 2 bytes (Low and High)
   uint8_t accel_buffer[6];
   int16_t gyro_x_raw,gyro_y_raw,gyro_z_raw;

	  set_servo_angle(&htim2, TIM_CHANNEL_1, 0);
	  set_servo_angle(&htim2, TIM_CHANNEL_2, 0);
	  set_servo_angle(&htim2, TIM_CHANNEL_3, 0);





  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  for(int i = 0; i < 2048; i++)
//	  	  	    {
//	  	  	        Move_Stepper(1);  // Take exactly ONE step
//	  	  	        HAL_Delay(2);     // Wait for the motor gears to move
//	  	  	    }
//
//	  	  	    HAL_Delay(200);      // Wait 1 second at the end of rotation
//
//	  	  	    // MOVE BACKWARD: 2048 steps back
//	  	  	    for(int i = 0; i < 2048; i++)
//	  	  	    {
//	  	  	        Move_Stepper(-1); // Take exactly ONE step backward
//	  	  	        HAL_Delay(2);
//	  	  	    }
//
//	  	  	    HAL_Delay(200);

	  HAL_I2C_Mem_Read(&hi2c2, 0xD5, 0x22, 1, buffer, 6, 100); // mem-read asks the chipset for the data
	  HAL_I2C_Mem_Read(&hi2c2, 0xD5, 0x28, 1, accel_buffer, 6, 100);

	  	  gyro_x_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
	  	  gyro_y_raw = (int16_t)((buffer[3] << 8) | buffer[2]);
	  	  gyro_z_raw = (int16_t)((buffer[5] << 8) | buffer[4]);


	  	  accel_x_raw = (int16_t)((accel_buffer[1] << 8) | accel_buffer[0]);
	  	  accel_y_raw = (int16_t)((accel_buffer[3] << 8) | accel_buffer[2]);
	  	  accel_z_raw = (int16_t)((accel_buffer[5] << 8) | accel_buffer[4]);


	  	  gyro_x_dps = (float)gyro_x_raw * 0.0175f;
	  	  gyro_y_dps = (float)gyro_y_raw * 0.0175f;
	  	  gyro_z_dps = (float)gyro_z_raw * 0.0175f;


	  	  //complementary filter for each servo :
			float accel_pitch = atan2f(accel_y_raw, accel_z_raw) * 57.2958f;
			float accel_roll  = atan2f(accel_x_raw, accel_z_raw) * 57.2958f;

			// 2. Apply Complementary Filter
			fused_pitch = 0.98f * (fused_pitch + gyro_x_dps * 0.02f) + 0.02f * accel_pitch;
			fused_roll  = 0.98f * (fused_roll  + gyro_y_dps * 0.02f) + 0.02f * accel_roll;
			// Yaw is purely gyro-based unless you add a magnetometer
			fused_yaw   = fused_yaw + gyro_z_dps * 0.02f;

			// 3. Servo Mixing Logic
			int s1_angle = 90 + (int)(fused_pitch + fused_roll);
			int s2_angle = 90 + (int)(fused_pitch - fused_roll);
			int s3_yaw   = 90 - (int)fused_yaw;

			// 4. Output to TIM2 PWM
			set_servo_angle(&htim2, TIM_CHANNEL_1, s1_angle);
			set_servo_angle(&htim2, TIM_CHANNEL_2, s2_angle);
			set_servo_angle(&htim2, TIM_CHANNEL_3, s3_yaw);

//	  	  printf("X: %.2f dps | Hex: 0x%04X \r\n Y: %.2f dps | Hex: 0x%04X \r\n Z: %.2f dps | Hex: 0x%04X \r\n", gyro_x_dps, (int16_t)gyro_x_dps
//	  			  ,gyro_y_dps, (int16_t)gyro_y_dps
//	  			  ,gyro_z_dps, (int16_t)gyro_z_dps);
	  	printf("[GYROSCOPE]/*%.2f,%.2f,%.2f*/\r\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);
	  	printf("[ACCEL]/*%.2f,%.2f,%.2f*/\r\n", accel_x_raw, accel_y_raw, accel_z_raw);

//		printf("Y: %.2f dps | Hex: 0x%04X \r\n", gyro_y_dps, (int16_t)gyro_y_dps);
//	  	  HAL_Delay(500);

//	  	  set_servo_angle(&htim2, TIM_CHANNEL_1, (-gyro_x_dps)*2);
//	  	  set_servo_angle(&htim2, TIM_CHANNEL_2, (-gyro_y_dps)*2);
//	  	  set_servo_angle(&htim2, TIM_CHANNEL_3, (-gyro_z_dps)*2);
//	  	  HAL_Delay(50); // Faster update for smooth movement



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10D19CE4;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1600;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
