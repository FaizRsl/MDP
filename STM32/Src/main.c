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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "ICM20948.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for showTask */
osThreadId_t showTaskHandle;
const osThreadAttr_t showTask_attributes = {
  .name = "showTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyroTask */
osThreadId_t gyroTaskHandle;
const osThreadAttr_t gyroTask_attributes = {
  .name = "gyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motor(void *argument);
void encoder(void *argument);
void servo(void *argument);
void gyroscope(void *argument);

/* USER CODE BEGIN PFP */
// void motor();
void motorStart();
void servoStart();
void motorDirection(int dir);
void move(void *argument);
void move_Distance(uint8_t dist, int pwm);
void forward_right(int angle);
void forward_left(int angle);
void reverse_right(int angle);
void reverse_left(int angle);
void forward();
void backward();
void algo_FR_indoor(int angle);
void algo_FL_indoor(int angle);
void algo_BR_indoor(int angle);
void algo_BL_indoor(int angle);
void algo_FR_outdoor(int angle);
void algo_FL_outdoor(int angle);
void algo_BR_outdoor(int angle);
void algo_BL_outdoor(int angle);
void gyroInit();
void HCSR04();
void delay(uint16_t time);
void writeByte(uint8_t addr,uint8_t data);
void readByte(uint8_t addr, uint8_t* data);
void small_obstacle();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* unused variables
int16_t error;           // error between target and actual
int32_t error_area = 0;  // area under error - to calculate I for PI implementation
int error_left, error_right;
float error_rate; // to calculate D for PID control
int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
uint32_t counter = 0;    // Timer 2 counter
int16_t count = 0;       // Convert counter to signed value
uint16_t position = 0;    // position of the motor (1 rotation = 260 count)
uint16_t angle = 0;
int target_tick= 500;
*/
uint16_t pwmMax = 1500;
float full_rotation = 1320;
float wheel_circum = 21.991; //wheel diameter 7cm
uint16_t pwmValA, pwmValB = 0;
uint8_t RxBuffer[10];
char rpiMsg[50];
uint8_t status[20];
char *control[] = {"FW", "BW", "FL", "FR", "BL", "BR", "OB", "TL", "TR"}; // movement controls for Android device
//Gyroscope variables
uint8_t ICM_ADDR = 0x68;
uint8_t buff[20]; //gyroscope buffer
double TURNING_ANGLE = 0;
double TOTAL_ANGLE = 0;

// HCSR04 variables
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;
#define TRIG_PIN GPIO_PIN_13
#define TRIG_PORT GPIOD
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  // strcpy(status,"Peripheral OK");
  HAL_UART_Receive_IT(&huart3,(uint8_t *) RxBuffer,4); //active for 4 bytes
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of showTask */
  showTaskHandle = osThreadNew(show, NULL, &showTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(move, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder, NULL, &EncoderTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(servo, NULL, &servoTask_attributes);

  /* creation of gyroTask */
  gyroTaskHandle = osThreadNew(gyroscope, NULL, &gyroTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  *
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SCLK_Pin|SDIN_Pin|RES_Pin|D_C_Pin
                          |LED3_Pin|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SCLK_Pin SDIN_Pin RES_Pin D_C_Pin
                           LED3_Pin PE1 */
  GPIO_InitStruct.Pin = SCLK_Pin|SDIN_Pin|RES_Pin|D_C_Pin
                          |LED3_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
		if(Is_First_Captured == 0){
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			Is_First_Captured = 1;
			// set polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(Is_First_Captured == 1){
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim,0);

			Distance = (IC_Val2 > IC_Val1 ? IC_Val2 - IC_Val1:(0xffff - IC_Val1)+IC_Val2) * 0.0340 / 2;
			Is_First_Captured = 0; //set it back to false

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);

		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	/*prevent unused argument(s)*/
	UNUSED(huart);

	HAL_UART_Transmit(&huart3, (uint8_t *)RxBuffer,4,0xFFFF);
	HAL_UART_Receive_IT(&huart3,(uint8_t *) RxBuffer,4);
}

void HCSR04(){ //PD13
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	delay(10); //10 microseconds
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);

}

void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < time);
}

void motorStart(){
	//generate PWM signal for Motor A
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	// generate PWM signal for Motor B
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
}

void servoStart(){
	// generate PWM signal for servo motor
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

void motorDirection(int dir)
{
  if(dir == 1){
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
  }
  else if(dir == -1){
	  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
  }
  else{
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
  }

}

void writeByte(uint8_t addr, uint8_t data)
{
  buff[0] = addr;
  buff[1] = data;
  HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR<<1, buff, 2, 20);
}

void readByte(uint8_t addr, uint8_t *data)
{
  buff[0] = addr;
  // Tell we want to read from the register
  HAL_I2C_Master_Transmit(&hi2c1, ICM_ADDR<<1, buff, 1, 10);
  // Read 2 byte from z dir register
  HAL_I2C_Master_Receive(&hi2c1, ICM_ADDR<<1, data, 2, 20);
}

void gyroInit(){
	 writeByte(0x06, 0x00);
	 osDelayUntil(10);
	 writeByte(0x03, 0x80);
	 osDelayUntil(10);
	 writeByte(0x07, 0x3F);
	 osDelayUntil(10);
	 writeByte(0x06, 0x01);
	 osDelayUntil(10);
	 writeByte(0x7F, 0x20); // go to bank 2
	 osDelayUntil(10);
	 writeByte(0x01, 0x2F); // config gyro, enable gyro, dlpf, set gyro to +-2000dps; gyro lpf = 3'b101
	 osDelayUntil(10);
	 writeByte(0x00, 0x00); // set gyro sample rate divider = 1 + 0(GYRO_SMPLRT_DIV[7:0])
	 osDelayUntil(10);
	 writeByte(0x01, 0x2F); // config accel, enable gyro, dlpf, set gyro to +-2000dps; gyro lpf = 3'b101
	 osDelayUntil(10);
	 writeByte(0x00, 0x00); // set gyro sample rate divider = 1 + 0(GYRO_SMPLRT_DIV[7:0])
	 osDelayUntil(10);
	 writeByte(0x7F, 0x00); // return to bank 1
	 osDelayUntil(10);
	 writeByte(0x07, 0x00);
	 osDelayUntil(10);
}

void move_Distance(uint8_t Dist, int pwm){
	uint8_t temp_dist = 0;
	uint32_t tick, total_Dist = 0;
	int lefttick, righttick = 0;
	int left_prev, left_after, right_prev, right_after = 0;
	// begin left motor encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Left
	// begin right motor encoder
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //Right
	left_prev = __HAL_TIM_GET_COUNTER(&htim2); //TIM2->CNT may work
	right_prev = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	// for lab

	/*
	pwmValA = 2000;
	pwmValB = 0.95 * pwmValA;
	*/
	pwmValA = pwm;
	pwmValB = 0.8125 * pwmValA;


	htim1.Instance->CCR4 = 148;

	while(total_Dist <= Dist){

	  //modify comparison value for PWM duty cycle
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	  osDelay(10);

	  if(HAL_GetTick() - tick > 100L){
			left_after = __HAL_TIM_GET_COUNTER(&htim2);
			right_after = __HAL_TIM_GET_COUNTER(&htim3);

			//left encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){

				if(left_after < left_prev){
					lefttick = left_prev - left_after;
				}
				else{
					lefttick = 0; //65535 is counter period
				}
			}
			else{
				if(left_after > left_prev){
					lefttick = left_after - left_prev;
				}
				else{
					lefttick = 0;
				}
			}

			//right encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
					if(right_after < right_prev){
						righttick = right_prev - right_after;
					}
					else{
						righttick = 0; //65535 is counter period
					}
				}
				else{
					if(right_after > right_prev){
						righttick = right_after - right_prev;
					}
					else{
						righttick = 0;
					}
				}

			temp_dist = (uint8_t)((lefttick/full_rotation)*wheel_circum);
			total_Dist += temp_dist;
			left_prev = __HAL_TIM_GET_COUNTER(&htim2);
			right_prev = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();
			osDelay(1);
		  }
	  }
	  motorDirection(0);
}

void move(void *argument){
	motorStart();
	servoStart();
	// Left 90 degree -> 60, 180 degree -> 120, 270 degree -> 182, 360 degree -> 255
	// Right 90 degree -> 60,
	htim1.Instance->CCR4 = 148; //center
	  // reverse_left(45);
	  // reverse_left(120);
	  // reverse_left(190);
	  // reverse_left(250);

	  // forward_left(50);
	  // forward_left(110);
	  // forward_left(180);
	  // forward_left(250);

	  //  forward_right(50);

	  // forward_right(110);
	  // forward_right(180);
	  // forward_right(250);

	  // reverse_right(45);
	  // reverse_right(120);
	  // reverse_right(180);
	  // reverse_right(250);

	forward();
	backward();
	//table & lab
	// 40 x 50

	/*
	forward_left(50);
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(400);
	motorDirection(0);
	*/

	/*
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(930);
	reverse_left(46);
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1500);
	motorDirection(0);
	*/

	/*
	forward_right(50);
	osDelay(100);
	// correction
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(360);
	motorDirection(0);
	*/

    /*
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1400);
	reverse_right(60);
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(940);
	motorDirection(0);
	*/

	// outside

	// 40 x 50
	/*
	motorDirection(1);
	forward_right(52);
	osDelay(100);
	// correction
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(460);
	motorDirection(0);
	*/

	/*
	 motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1400);
	reverse_right(60);
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(940);
	motorDirection(0);
	*/


	// must change forward_left() angle for indoor and outdoor
	/*
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 975);
	osDelay(250);
	forward_left(45);
	*/

	/*
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(500);
	reverse_left(48);
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(600);
	motorDirection(0);
	*/
}


// 10cm
void forward(){ //called when string FW is detected

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);
	motorStart();
	servoStart();
	htim1.Instance->CCR4 = 148; //center
	motorDirection(1);
	osDelay(50);
	move_Distance(6, 2000);
}

void backward(){ //called when string BW is called
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);
	motorStart();
	servoStart();
	htim1.Instance->CCR4 = 148; //center
	motorDirection(-1);
	osDelay(50);
	move_Distance(6, 2000);
}

void forward_right(int angle){

	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
    double prev_angle;

    uint8_t val[2] = {0, 0};
    int16_t angular_speed = 0;
    uint32_t tick = 0;
    gyroInit();

    tick = HAL_GetTick();
    osDelayUntil(10);

    //turn right
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);
	motorStart();
	servoStart();

	htim1.Instance->CCR4 = 225;
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE > -1*angle){ // gyro rotate clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 225;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(1000);
}

void forward_left(int angle){
	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//turn left
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	motorStart();
	servoStart();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);

	// htim1.Instance->CCR4 = 115; // indoor
	htim1.Instance->CCR4 = 105; // outdoor
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE < angle){ //gyro rotate counter-clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;


		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		// htim1.Instance->CCR4 = 115; // indoor
		htim1.Instance->CCR4 = 105;
		//outdoor
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(1000);
}

void reverse_right(int angle){
	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//reverse right

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	motorStart();
	servoStart();

	htim1.Instance->CCR4 = 240;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE < angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 240;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(1000);
}

void reverse_left(int angle){
	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	motorStart();
	servoStart();

	htim1.Instance->CCR4 = 100;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE > -1*angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 100;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(1000);
}

// original in move()
void algo_FR_indoor(int angle){
	// indoor

	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
    double prev_angle;

    uint8_t val[2] = {0, 0};
    int16_t angular_speed = 0;
    uint32_t tick = 0;
    gyroInit();

    tick = HAL_GetTick();
    osDelayUntil(10);

    //turn right
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);

	htim1.Instance->CCR4 = 225;
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE > -1*angle){ // gyro rotate clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 225;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;


	osDelay(100);
	// correction
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(360);
	motorDirection(0);

}
void algo_FL_indoor(int angle){
	// indoor

	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//turn left
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);

	htim1.Instance->CCR4 = 115; // indoor
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE < angle){ //gyro rotate counter-clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;


		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 115; // indoor
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(100);

	// correction
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(400);
	motorDirection(0);

}
void algo_BR_indoor(int angle){
	// indoor
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1400);


	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//reverse right

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;

	htim1.Instance->CCR4 = 240;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE < angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 240;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;

	// correction
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(940);
	motorDirection(0);

}
void algo_BL_indoor(int angle){
	// indoor

	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(930);


	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;

	htim1.Instance->CCR4 = 100;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE > -1*angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 100;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(100);

	// correction
	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1500);
	motorDirection(0);

}

void algo_FR_outdoor(int angle){
	motorDirection(1);

	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();

	tick = HAL_GetTick();
	osDelayUntil(10);

	//turn right
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0);

	htim1.Instance->CCR4 = 225;
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE > -1*angle){ // gyro rotate clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 225;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;


	osDelay(100);
	// correction
	motorDirection(-1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(460);
	motorDirection(0);
}
void algo_FL_outdoor(int angle){
	// must change forward_left() angle for indoor and outdoor
	// MUST CHANGE CCR4 VALUE
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 975);
	osDelay(250);

	// Gyroscope
	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//turn left
	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);

	htim1.Instance->CCR4 = 105; // outdoor
	motorDirection(1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE < angle){ //gyro rotate counter-clockwise
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;


		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 105;
		//outdoor
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;
	osDelay(100);

}
void algo_BR_outdoor(int angle){
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(1400);

	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	//reverse right

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;

	htim1.Instance->CCR4 = 240;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,(pwmValB/2));
	while(__TURNING_ANGLE < angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 240;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;

	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(940);
	motorDirection(0);
}
void algo_BL_outdoor(int angle){
	motorDirection(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(500);

	double offset = 0; //7.848882995
	double temp_angle;
	double prev_angle;

	uint8_t val[2] = {0, 0};
	int16_t angular_speed = 0;
	uint32_t tick = 0;
	gyroInit();
	tick = HAL_GetTick();
	osDelayUntil(10);

	pwmValA = 1500;
	pwmValB = 1.01 * pwmValA;

	htim1.Instance->CCR4 = 100;
	motorDirection(-1);
	osDelay(500);
	int __TURNING_ANGLE = 0;

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA/2);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	while(__TURNING_ANGLE > -1*angle){
		osDelay(10);
		if (HAL_GetTick() - tick >= 50L)
		{
		  readByte(0x37, val);
		  angular_speed = (val[0] << 8) | val[1];

		  temp_angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		  TOTAL_ANGLE += temp_angle;
		  __TURNING_ANGLE += temp_angle;

		  if (TOTAL_ANGLE >= 720)
		  {
			TOTAL_ANGLE -= 720;
		  }
		  if (TOTAL_ANGLE <= -720)
		  {
			TOTAL_ANGLE += 720;
		  }

		  tick = HAL_GetTick();
		  prev_angle = temp_angle;
		}
		osDelay(1);
		htim1.Instance->CCR4 = 100;
	}
	motorDirection(0);
	htim1.Instance->CCR4 = 148;


	osDelay(100);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,1200);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,975);
	osDelay(600);
	motorDirection(0);
}


void small_obstacle(){ //10cm x 10cm
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1);
	motorStart();
	servoStart();
	pwmValA = 2000;
	pwmValB = pwmValA;
	motorDirection(-1);
	htim1.Instance->CCR4 = 148;
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	osDelay(1000);

	forward_left(23); // 45 degree left
	osDelay(250);
	forward_right(75); // 45 degree right
	motorDirection(0);


}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  // Turning angle will change depending on the battery level
   uint8_t command[10], motion[3], x[3], y[2] = "\0";
   uint8_t display[5];
   int x_int, y_int, i;

   motorStart();
   servoStart();
   /* Infinite loop */
   for(;;){
	   sprintf(command, "%s", RxBuffer);
	   for(int j=0; j<2; j++){ motion[j] = command[j]; }
	   motion[2] = '\0';
	   strcpy((char *)x, &command[2]);
	   strcpy((char *)y, &command[3]);
	   x_int = (atoi((char *)x)/10)%10;
	   y_int = atoi((char *)y);

	   // OLED_ShowString(0,5,command);
	   OLED_ShowString(0,30,motion);
	   sprintf(display,"x:%d", x_int);

	   OLED_ShowString(40,30,display);

	if(strcmp((char *)motion, control[0]) == 0){
		for(i=0;i<x_int;i++){
			forward();
		}
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[1]) == 0 ){
		for(i=0;i<x_int;i++){
			backward();
		}
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[2]) == 0){
		// algo_FL_indoor(50);
		algo_FL_indoor(47); //TR
		// algo_FL_outdoor(45);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[3]) == 0){
		algo_FR_indoor(48); //TR
		//algo_FR_indoor(50);
		// algo_FR_outdoor(52);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[4]) == 0){
		algo_BL_indoor(51); //TR
		//algo_BL_indoor(46);
		// algo_BL_outdoor(48);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[5]) == 0){
		algo_BR_indoor(54);
		// algo_BR_outdoor(60);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}

	else if(strcmp((char *)motion, control[6]) == 0){
		small_obstacle();
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n",6,0xFFFF);
	}
	
	else if(strcmp((char *)motion, control[7]) == 0){
		algo_FL_indoor(50);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n", 6, 0xFFFF);
	}
	else if(strcmp((char *)motion, control[8]) == 0){
		algo_FR_indoor(48);
		HAL_UART_Transmit(&huart3, (uint8_t *)"ACK|\r\n", 6, 0xFFFF);
	}


	strcpy(RxBuffer, "");
	OLED_Refresh_Gram();
	osDelay(500); // might need to decrease during actual implementation
   }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the showTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	uint8_t alphabet = 'W';
	uint8_t display[10];

	  /* Infinite loop */
	  for(;;)
	  {
		  HCSR04();
		  sprintf(display,"%d",Distance);
		  OLED_ShowString(40,40,display);
		  HAL_UART_Transmit(&huart3,(uint8_t *)&alphabet,1,0xFFFF);
		  if(alphabet < 'Z'){
			  alphabet++;
		  }
		  else{
			  alphabet = 'A';
		  }
		  // HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		  osDelay(1000);
	  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  /* Infinite loop */
	uint8_t temp_dist = 0;
	uint32_t tick, total_Dist = 0;
	int lefttick, righttick = 0;
	int left_prev, left_after, right_prev, right_after = 0;

	//generate PWM signal for servo motor
	servoStart();
	//generate PWM signal for Motor A & B
	motorStart();
	// begin left motor encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Left
	// begin right motor encoder
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //Right
	left_prev = __HAL_TIM_GET_COUNTER(&htim2); //TIM2->CNT may work
	right_prev = __HAL_TIM_GET_COUNTER(&htim3);
	tick = HAL_GetTick();
	// for lab
	pwmValA = 2000;
	pwmValB = 0.95 * pwmValA;
	htim1.Instance->CCR4 = 148;

    while(total_Dist <= 80){

	 // forward
	  motorDirection(1);
	  //modify comparison value for PWM duty cycle
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1,pwmValA);
	  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2,pwmValB);
	  osDelay(10);

	  if(HAL_GetTick() - tick > 400L){
			left_after = __HAL_TIM_GET_COUNTER(&htim2);
			right_after = __HAL_TIM_GET_COUNTER(&htim3);

			//left encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){

				if(left_after < left_prev){
					lefttick = left_prev - left_after;
				}
				else{
					lefttick = 0; //65535 is counter period
				}
			}
			else{
				if(left_after > left_prev){
					lefttick = left_after - left_prev;
				}
				else{
					lefttick = 0;
				}
			}

			//right encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
					if(right_after < right_prev){
						righttick = right_prev - right_after;
					}
					else{
						righttick = 0; //65535 is counter period
					}
				}
				else{
					if(right_after > right_prev){
						righttick = right_after - right_prev;
					}
					else{
						righttick = 0;
					}
				}

			/*PID Control
			error_left = target_tick - lefttick;
			error_right = target_tick - righttick;
			pwmValA = error_left * Kp;
			pwmValB = error_right * Kp;

			if(pwmValA > pwmMax || pwmValB > pwmMax){
				pwmValA = pwmMax;
				pwmValB = pwmMax;
			}
			pwmValA = (uint8_t)PIDController_Update(&PWM_L, lefttick, 2000 ,pwmValA);
			pwmValB = (uint8_t)PIDController_Update(&PWM_R, righttick, 2000, pwmValB);
			*/

			temp_dist = (uint8_t)((lefttick/full_rotation)*wheel_circum);
			total_Dist += temp_dist;
			left_prev = __HAL_TIM_GET_COUNTER(&htim2);
			right_prev = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();
			osDelay(1);

		  }
	  }
      motorDirection(0);
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  /* Infinite loop */
	  int dir;
	  int cnt_1, cnt_2, cnt_3, cnt_4;
	  float diffLeft, diffRight = 0;
	  uint16_t total_Dist = 0;
	  uint8_t display[50], temp_dist;
	  uint32_t tick;
	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //Left
	  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //Right
	  //increase by one whenever rising edge
	  cnt_1 = __HAL_TIM_GET_COUNTER(&htim2); //TIM2->CNT may work
	  cnt_3 = __HAL_TIM_GET_COUNTER(&htim3);
	  tick = HAL_GetTick();
	  for(;;)
	  {
		if(HAL_GetTick() - tick > 500L){
			cnt_2 = __HAL_TIM_GET_COUNTER(&htim2);
			cnt_4 = __HAL_TIM_GET_COUNTER(&htim3);

			//left encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){
				dir = 1;
				if(cnt_2 < cnt_1){
					diffLeft = cnt_1 - cnt_2;
				}
				else{
					// diffLeft = (65535 - cnt_2) + cnt_1; //65535 is counter period
					diffLeft = 0;
				}
			}
			else{
				dir = -1;
				if(cnt_2 > cnt_1){
					diffLeft = cnt_2 - cnt_1;
				}
				else{
					// diffLeft = (65535 - cnt_1) + cnt_2;
					diffLeft = 0;
				}
			}

			//right encoder
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){
					if(cnt_4 < cnt_3){
						diffRight = cnt_3 - cnt_4;
					}
					else{
						// diffRight = (65535 - cnt_4) + cnt_3; //65535 is counter period
						diffRight = 0;
					}
				}
				else{
					if(cnt_4 > cnt_3){
						diffRight = cnt_4 - cnt_3;
					}
					else{
						// diffRight = (65535 - cnt_3) + cnt_4;
						diffRight = 0;
					}
				}

			// dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			temp_dist = (uint8_t)((diffLeft/full_rotation)*wheel_circum);
			total_Dist += temp_dist;
			/*
			sprintf(display,"Left: %2d", diffLeft);
			OLED_ShowString(0,20,display);
			sprintf(display, "Right: %2d", (uint8_t)diffRight);
			OLED_ShowString(0,30,display);
			*/
			sprintf(display,"Temp:%d",temp_dist);
			OLED_ShowString(30,40,display);
			sprintf(display,"D:%d", total_Dist);
			OLED_ShowString(0,40,display);
			cnt_1 = __HAL_TIM_GET_COUNTER(&htim2);
			cnt_3 = __HAL_TIM_GET_COUNTER(&htim3);
			tick = HAL_GetTick();

		}
		osDelay(1);
	  }
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_servo */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo */
void servo(void *argument)
{
  /* USER CODE BEGIN servo */
  /* Infinite loop */
  servoStart();
  for(;;)
{
	htim1.Instance->CCR4 = 240;
	osDelay(2000);
    htim1.Instance->CCR4 = 210; //r
    osDelay(2000);
    htim1.Instance->CCR4 = 180; //c
    osDelay(2000);
    htim1.Instance->CCR4 = 150; //c
	osDelay(2000);
	htim1.Instance->CCR4 = 120; //c
	osDelay(2000);
	htim1.Instance->CCR4 = 90; //extreme left
	osDelay(2000);
	htim1.Instance->CCR4 = 150;
	osDelay(2000);
  }
  /* USER CODE END servo */
}

/* USER CODE BEGIN Header_gyroscope */
/**
* @brief Function implementing the gyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyroscope */
void gyroscope(void *argument)
{
  /* USER CODE BEGIN gyroscope */
  /* Infinite loop */
	uint8_t display[20];
	  double offset = 0; //7.848882995
	  double angle;
	  double prev_angle;
	  for(;;)
	  {
		  uint8_t val[2] = {0, 0};
		  int16_t angular_speed = 0;

		  uint32_t tick = 0;
		  gyroInit();

		  tick = HAL_GetTick();
		  osDelayUntil(10);

		  for (;;)
		  {

		    osDelay(10);
		    if (HAL_GetTick() - tick >= 50L)
		    {
		      readByte(0x37, val);
		      angular_speed = (val[0] << 8) | val[1];



		      angle = 0.1*(((double)(angular_speed)+offset) * ((HAL_GetTick() - tick) / 16400.0)) + 0.9*(prev_angle);

		      TOTAL_ANGLE += angle;
		      TURNING_ANGLE += angle;


		      if (TOTAL_ANGLE >= 720)
		      {
		        TOTAL_ANGLE -= 720;
		      }
		      if (TOTAL_ANGLE <= -720)
		      {
		        TOTAL_ANGLE += 720;
		      }
		      sprintf(display, "ANGLE: %6d\0", (int)(TURNING_ANGLE));
		      // sprintf(display, "A_100: %6d\0", (int)(100*TOTAL_ANGLE));
		      OLED_ShowString(0,20,display);
		      tick = HAL_GetTick();
		      prev_angle = angle;
		    }
		    osDelay(1);
		  }
	  }
  /* USER CODE END gyroscope */
}

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
