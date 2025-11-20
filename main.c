/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_ADDR (0x68)
#define B_IMU_ADDR (0x4A)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void usr_print_num(int);
void usr_print_str(char *usr_str);
void define_pwm_freq(int usr_freq, int clock_freq);
void motor_calibration(int min_val, int max_val);
void return_BNO085_data(int16_t* BNO085_data);
void proportional_derivative_stabilization_roll(int16_t roll_val, int16_t error_val);/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define freq
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  usr_print_num(1);
  HAL_Delay(2000);
  usr_print_num(2);
  HAL_Delay(2000);
  usr_print_num(3);
  HAL_Delay(2000);

  //yaw, roll, pitch, x-accel, y-accel, z-accel
  int16_t BNO085[6];
  int16_t current_error = 0;
  int16_t previous_error = 0;
  int16_t derivative_error = 0;

//  motor_calibration(1000, 2000);
  TIM2->CCR1 = 1100;
  TIM2->CCR2 = 1100;
  TIM2->CCR3 = 1100;
  TIM2->CCR4 = 1100;

  HAL_Delay(3000);
  TIM2->CCR1 = 1000;
  TIM2->CCR2 = 1000;
  TIM2->CCR3 = 1000;
  TIM2->CCR4 = 1000;
  HAL_Delay(3000);

//   uint8_t data;
//   data = 0x00;
//   HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//   data = 0x01;
//   HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//   uint8_t who_am_i = 0;
//   HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);
//   usr_print_num(who_am_i);

  /* Get initial reading and seed the error */
  return_BNO085_data(BNO085);
  int16_t initial_roll = BNO085[2];   // roll from IMU buffer
  previous_error = -initial_roll;     // target roll = 0 deg

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  return_BNO085_data(BNO085);
//	  usr_print_num(BNO085[1]);

	  int16_t roll = BNO085[2];          // use roll axis
	  current_error = -roll;             // error = target(0) - roll
	  derivative_error = current_error - previous_error;

	  proportional_derivative_stabilization_roll(roll, derivative_error);
	  previous_error = current_error;

//	  TIM2->CCR2 = 1250;
//	  HAL_I2C_Mem_Read((&hi2c1), (IMU_ADDR), (accel_x_low_addr), (I2C_MEMADD_SIZE_8BIT), &accel_x_low, 1, 100);
//	  HAL_I2C_Mem_Read((&hi2c1), (IMU_ADDR), (accel_x_high_addr), (I2C_MEMADD_SIZE_8BIT), &accel_x_high, 1, 100);
//	  accel_x = (accel_x_high << 8) | (accel_x_low);
//	  usr_print_num(accel_x);


//	  HAL_UART_Receive(&huart1, rx_buffer, 19, 100);
//	  HAL_UART_Receive(&huart1, &header_1, 1, 100);
//	  if (header_1 == 0xAA){
//		  HAL_UART_Receive(&huart1, &header_2, 1, 100);
//		  if (header_1 == 0xAA){
//
//			  HAL_UART_Receive(&huart1, &index, 1, 100);
//			  HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &yaw_2, 1, 100);
//
//			  HAL_UART_Receive(&huart1, &pitch_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &pitch_2, 1, 100);
//
//			  HAL_UART_Receive(&huart1, &roll_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &roll_2, 1, 100);
//
//			  yaw = ((yaw_2<<8) | (yaw_1));
//			  pitch = ((pitch_2<<8) | (pitch_1));
//			  roll = ((roll_2<<8) | (roll_1));
//
//			  usr_print_num(index);
//			  usr_print_num(yaw);
//			  usr_print_num(pitch);
//			  usr_print_num(roll);
//		  }
//	  }



//	  for (int i=1000; i<=2000; i++){
//		  TIM2->CCR1 = i;
//		  HAL_Delay(10);
//	  }
//	  for (int i=2000; i>=1000; i--){
//	  		  TIM2->CCR1 = i;
//	  		  HAL_Delay(10);
//	  	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void usr_print_str(char *usr_str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)usr_str, strlen(usr_str), 100);
}

void usr_print_num(int usr_num)
{
	char usr_str[20];
	sprintf(usr_str, "%d\r\n", usr_num);
	HAL_UART_Transmit(&huart2, (uint8_t*)usr_str, strlen(usr_str), 100);
}

void define_pwm_freq(int usr_freq, int clock_freq)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = clock_freq - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = (1000000 / usr_freq);
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

}

void motor_calibration(int min_val, int max_val){
	TIM2->CCR1 = max_val;
	TIM2->CCR2 = max_val;
	TIM2->CCR3 = max_val;
	TIM2->CCR4 = max_val;
	HAL_Delay(10000);
	TIM2->CCR1 = min_val;
	TIM2->CCR2 = min_val;
	TIM2->CCR3 = min_val;
	TIM2->CCR4 = min_val;
	usr_print_num(2);
	HAL_Delay(20000);
	usr_print_num(3);
}

void wake_BNO085(){
	uint8_t data;
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	uint8_t who_am_i = 0;
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);
	usr_print_num(who_am_i);
}

void return_BNO085_data(int16_t* BNO085_data){

	int16_t yaw;
	uint8_t yaw_1;
	uint8_t yaw_2;

	int16_t pitch;
	uint8_t pitch_1;
	uint8_t pitch_2;

	int16_t roll;
	uint8_t roll_1;
	uint8_t roll_2;

	int16_t x_accel;
	uint8_t x_accel_1;
	uint8_t x_accel_2;

	int16_t y_accel;
	uint8_t y_accel_1;
	uint8_t y_accel_2;

	int16_t z_accel;
	uint8_t z_accel_1;
	uint8_t z_accel_2;

	uint8_t header_1;
	uint8_t header_2;

	while (1){
		HAL_UART_Receive(&huart1, &header_1, 1, 100);
		if (header_1 == 0xAA){
			HAL_UART_Receive(&huart1, &header_2, 1, 100);
			if (header_2 == 0xAA){

				HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
				HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
				HAL_UART_Receive(&huart1, &yaw_2, 1, 100);

				HAL_UART_Receive(&huart1, &pitch_1, 1, 100);
				HAL_UART_Receive(&huart1, &pitch_2, 1, 100);

				HAL_UART_Receive(&huart1, &roll_1, 1, 100);
				HAL_UART_Receive(&huart1, &roll_2, 1, 100);

				HAL_UART_Receive(&huart1, &x_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &x_accel_2, 1, 100);

				HAL_UART_Receive(&huart1, &y_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &y_accel_2, 1, 100);

				HAL_UART_Receive(&huart1, &z_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &z_accel_2, 1, 100);

				yaw     = ((yaw_2     << 8) | (yaw_1));
				pitch   = ((pitch_2   << 8) | (pitch_1));
				roll    = ((roll_2    << 8) | (roll_1));
				x_accel = ((x_accel_2 << 8) | (x_accel_1));
				y_accel = ((y_accel_2 << 8) | (y_accel_1));
				z_accel = ((z_accel_2 << 8) | (z_accel_1));

				BNO085_data[0] = yaw;
				BNO085_data[1] = pitch;
				BNO085_data[2] = roll;
				BNO085_data[3] = x_accel;
				BNO085_data[4] = y_accel;
				BNO085_data[5] = z_accel;
				break;

			}
		}
	}
}

void proportional_derivative_stabilization_roll(int16_t roll_val, int16_t error_val){
	int16_t proportional_val;
	int16_t derivative_val;

	if (abs(error_val) > 600){
		derivative_val = 0;
	}
	else{
		/* proper float multiply, then cast */
		derivative_val = (int16_t)(0.02f * (float)error_val);
	}

	int16_t base_val = 1500;

	if (roll_val >= 0){
		proportional_val = (int16_t)(0.02f * (float)roll_val);

		TIM2->CCR1 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR2 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR3 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR4 = (uint16_t)(base_val - derivative_val);
	}
	else{
		roll_val = (int16_t)(-roll_val);
		proportional_val = (int16_t)(0.02f * (float)roll_val);

		TIM2->CCR1 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR2 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR3 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR4 = (uint16_t)(base_val + proportional_val);
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
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IMU_ADDR (0x68)
#define B_IMU_ADDR (0x4A)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void usr_print_num(int);
void usr_print_str(char *usr_str);
void define_pwm_freq(int usr_freq, int clock_freq);
void motor_calibration(int min_val, int max_val);
void return_BNO085_data(int16_t* BNO085_data);
void proportional_derivative_stabilization_roll(int16_t roll_val, int16_t error_val);/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define freq
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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  usr_print_num(1);
  HAL_Delay(2000);
  usr_print_num(2);
  HAL_Delay(2000);
  usr_print_num(3);
  HAL_Delay(2000);

  //yaw, roll, pitch, x-accel, y-accel, z-accel
  int16_t BNO085[6];
  int16_t current_error = 0;
  int16_t previous_error = 0;
  int16_t derivative_error = 0;

//  motor_calibration(1000, 2000);
  TIM2->CCR1 = 1100;
  TIM2->CCR2 = 1100;
  TIM2->CCR3 = 1100;
  TIM2->CCR4 = 1100;

  HAL_Delay(3000);
  TIM2->CCR1 = 1000;
  TIM2->CCR2 = 1000;
  TIM2->CCR3 = 1000;
  TIM2->CCR4 = 1000;
  HAL_Delay(3000);

//   uint8_t data;
//   data = 0x00;
//   HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//   data = 0x01;
//   HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
//
//   uint8_t who_am_i = 0;
//   HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);
//   usr_print_num(who_am_i);

  /* Get initial reading and seed the error */
  return_BNO085_data(BNO085);
  int16_t initial_roll = BNO085[2];   // roll from IMU buffer
  previous_error = -initial_roll;     // target roll = 0 deg

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  return_BNO085_data(BNO085);
//	  usr_print_num(BNO085[1]);

	  int16_t roll = BNO085[2];          // use roll axis
	  current_error = -roll;             // error = target(0) - roll
	  derivative_error = current_error - previous_error;

	  proportional_derivative_stabilization_roll(roll, derivative_error);
	  previous_error = current_error;

//	  TIM2->CCR2 = 1250;
//	  HAL_I2C_Mem_Read((&hi2c1), (IMU_ADDR), (accel_x_low_addr), (I2C_MEMADD_SIZE_8BIT), &accel_x_low, 1, 100);
//	  HAL_I2C_Mem_Read((&hi2c1), (IMU_ADDR), (accel_x_high_addr), (I2C_MEMADD_SIZE_8BIT), &accel_x_high, 1, 100);
//	  accel_x = (accel_x_high << 8) | (accel_x_low);
//	  usr_print_num(accel_x);


//	  HAL_UART_Receive(&huart1, rx_buffer, 19, 100);
//	  HAL_UART_Receive(&huart1, &header_1, 1, 100);
//	  if (header_1 == 0xAA){
//		  HAL_UART_Receive(&huart1, &header_2, 1, 100);
//		  if (header_1 == 0xAA){
//
//			  HAL_UART_Receive(&huart1, &index, 1, 100);
//			  HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &yaw_2, 1, 100);
//
//			  HAL_UART_Receive(&huart1, &pitch_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &pitch_2, 1, 100);
//
//			  HAL_UART_Receive(&huart1, &roll_1, 1, 100);
//			  HAL_UART_Receive(&huart1, &roll_2, 1, 100);
//
//			  yaw = ((yaw_2<<8) | (yaw_1));
//			  pitch = ((pitch_2<<8) | (pitch_1));
//			  roll = ((roll_2<<8) | (roll_1));
//
//			  usr_print_num(index);
//			  usr_print_num(yaw);
//			  usr_print_num(pitch);
//			  usr_print_num(roll);
//		  }
//	  }



//	  for (int i=1000; i<=2000; i++){
//		  TIM2->CCR1 = i;
//		  HAL_Delay(10);
//	  }
//	  for (int i=2000; i>=1000; i--){
//	  		  TIM2->CCR1 = i;
//	  		  HAL_Delay(10);
//	  	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
  /* USER CODE END 3 */
}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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

int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void usr_print_str(char *usr_str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)usr_str, strlen(usr_str), 100);
}

void usr_print_num(int usr_num)
{
	char usr_str[20];
	sprintf(usr_str, "%d\r\n", usr_num);
	HAL_UART_Transmit(&huart2, (uint8_t*)usr_str, strlen(usr_str), 100);
}

void define_pwm_freq(int usr_freq, int clock_freq)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = clock_freq - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = (1000000 / usr_freq);
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

}

void motor_calibration(int min_val, int max_val){
	TIM2->CCR1 = max_val;
	TIM2->CCR2 = max_val;
	TIM2->CCR3 = max_val;
	TIM2->CCR4 = max_val;
	HAL_Delay(10000);
	TIM2->CCR1 = min_val;
	TIM2->CCR2 = min_val;
	TIM2->CCR3 = min_val;
	TIM2->CCR4 = min_val;
	usr_print_num(2);
	HAL_Delay(20000);
	usr_print_num(3);
}

void wake_BNO085(){
	uint8_t data;
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x7F, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	data = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, IMU_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);

	uint8_t who_am_i = 0;
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY);
	usr_print_num(who_am_i);
}

void return_BNO085_data(int16_t* BNO085_data){

	int16_t yaw;
	uint8_t yaw_1;
	uint8_t yaw_2;

	int16_t pitch;
	uint8_t pitch_1;
	uint8_t pitch_2;

	int16_t roll;
	uint8_t roll_1;
	uint8_t roll_2;

	int16_t x_accel;
	uint8_t x_accel_1;
	uint8_t x_accel_2;

	int16_t y_accel;
	uint8_t y_accel_1;
	uint8_t y_accel_2;

	int16_t z_accel;
	uint8_t z_accel_1;
	uint8_t z_accel_2;

	uint8_t header_1;
	uint8_t header_2;

	while (1){
		HAL_UART_Receive(&huart1, &header_1, 1, 100);
		if (header_1 == 0xAA){
			HAL_UART_Receive(&huart1, &header_2, 1, 100);
			if (header_2 == 0xAA){

				HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
				HAL_UART_Receive(&huart1, &yaw_1, 1, 100);
				HAL_UART_Receive(&huart1, &yaw_2, 1, 100);

				HAL_UART_Receive(&huart1, &pitch_1, 1, 100);
				HAL_UART_Receive(&huart1, &pitch_2, 1, 100);

				HAL_UART_Receive(&huart1, &roll_1, 1, 100);
				HAL_UART_Receive(&huart1, &roll_2, 1, 100);

				HAL_UART_Receive(&huart1, &x_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &x_accel_2, 1, 100);

				HAL_UART_Receive(&huart1, &y_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &y_accel_2, 1, 100);

				HAL_UART_Receive(&huart1, &z_accel_1, 1, 100);
				HAL_UART_Receive(&huart1, &z_accel_2, 1, 100);

				yaw     = ((yaw_2     << 8) | (yaw_1));
				pitch   = ((pitch_2   << 8) | (pitch_1));
				roll    = ((roll_2    << 8) | (roll_1));
				x_accel = ((x_accel_2 << 8) | (x_accel_1));
				y_accel = ((y_accel_2 << 8) | (y_accel_1));
				z_accel = ((z_accel_2 << 8) | (z_accel_1));

				BNO085_data[0] = yaw;
				BNO085_data[1] = pitch;
				BNO085_data[2] = roll;
				BNO085_data[3] = x_accel;
				BNO085_data[4] = y_accel;
				BNO085_data[5] = z_accel;
				break;

			}
		}
	}
}

void proportional_derivative_stabilization_roll(int16_t roll_val, int16_t error_val){
	int16_t proportional_val;
	int16_t derivative_val;

	if (abs(error_val) > 600){
		derivative_val = 0;
	}
	else{
		/* proper float multiply, then cast */
		derivative_val = (int16_t)(0.02f * (float)error_val);
	}

	int16_t base_val = 1500;

	if (roll_val >= 0){
		proportional_val = (int16_t)(0.02f * (float)roll_val);

		TIM2->CCR1 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR2 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR3 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR4 = (uint16_t)(base_val - derivative_val);
	}
	else{
		roll_val = (int16_t)(-roll_val);
		proportional_val = (int16_t)(0.02f * (float)roll_val);

		TIM2->CCR1 = (uint16_t)(base_val + proportional_val);
		TIM2->CCR2 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR3 = (uint16_t)(base_val - derivative_val);
		TIM2->CCR4 = (uint16_t)(base_val + proportional_val);
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
