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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// define size of shiftarray
#define SHIFT_ARRAY 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

//Analog input var's
uint32_t analogInputs[4];
uint32_t pulseDelayAvg;
uint32_t pulseWidthAvg;

//Shift register parameter's
int VOLTAGE_SHIFT = 10;
int CURRENT_SHIFT = 10;
int TEMP_SHIFT = 10;
int DELAY_SHIFT = 20;
int WIDTH_SHIFT = 20;
int RPM_SHIFT = 10;
int CYCLETIME_SHIFT = 5;

//Shift registers var's
uint32_t analogShift[SHIFT_ARRAY][5];
float rpmShift[SHIFT_ARRAY];
float cycleShift[SHIFT_ARRAY][2];
uint32_t voltageAvgCalc;
uint32_t currentAvgCalc;
uint32_t tempAvgCalc;
uint32_t pulseDelayAvgCalc;
uint32_t pulseWidthAvgCalc;
uint32_t pulseWidthAvgCalc;
float rpmAvgCalc;

//Voltage reading parameters
const float R1 = 29290.0;
const float R2 = 1000.0;

//Voltage reading var's
const float maxVoltage = (3.3 * ((R1 + R2) / R2));
const float voltageMultiplier = (maxVoltage / 4096.0); 

//Current reading parameters
const float maxCurrent = 10.0;

//Current reading var's
const float currentMultiplier = (maxCurrent / 4096.0); 

// TODO TEMPERATURE
const float tempMultiplier = 1;//(maxTemperature / 4096.0); 

//Power var's
float motorVoltage;
float motorCurrent;
float motorTemperature;

//Motor parameters
float uniPolePass = 3.0;

//Motor control
int motorEnable;
int pidEnable;
int pidAvgInput;
int uartEnable;
int cycleBufferEnable;
uint32_t delaySetpoint = 0;
uint32_t widthSetpoint = 1000;
float rpmSetPulse = 1000;

//Motor control var's
uint32_t pulseDelay = 0;
uint32_t pulseWidth = 500;

//Motor pulse var's
uint32_t cycleTimeRaw;
uint32_t halfTimeRaw;
uint32_t cycleTime;
uint32_t halfTime;
uint32_t cycleAvgCalc;
uint32_t halfCycleAvgCalc;
int highPulseState;
int lowPulseState;
float rpmPulse;
float hertzPulse;
uint32_t voltageAvg;
uint32_t currentAvg;
uint32_t tempAvg;
float rpmAvg;

//Motor state var's
int prevMotorEnable;
int pulseCycle;
int motorRunning;
int motorAtSpeed;

//Motor ramp up/down parameters
float rpmPulseAttackTime = 100;

//Motor ramp up/down var's
float rpmActSetPulse;
float rpmSetPulsePrev;
float rpmPulseIncremental;
int incrementalPulseDone;

//Motor startup parameters
uint32_t startupSetpoint = 1000000;
uint32_t startupFrequencyDecr = 1000;
float minimumMotorFreq = 6.0;

//Motor startup var's
uint32_t startupFrequency;
int motorStartupState;

//PID calculation parameters
float kp = 7.5;
float ki = 0.001;
float kd = 0.1;  
float PID_MIN = 1.0;
float PID_MAX = 900.0;
float maxRpmSetpoint = 4000.0;

//PID calculation var's
float error;
float cumError;
float rateError;
float lastError; 
float multiplierPid;

//UART var's
uint8_t Rx_data[16];
int rxDataAvailable;

//UART bus setable
int serialMotorEnable;
uint32_t serialDelaySetpoint;
uint32_t serialWidthSetpoint;
float serialRpmSetpoint;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

static void rxDataProcessing(void);
static void PIDcalculations(void);
static void RPMmotorRamp(void);
static void motorStartup(void);
static void motorRunningState(void);
static void rpmShifter(void);
static void analogShifter(void);
static void dataSelector(void);
static void powerCalculations(void);
static void motorCalculations(void);
static void motorEnableSafe(void);

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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	// start the time
	HAL_TIM_Base_Start(&htim5);
	
	// set enable output low
	GPIOC->BSRR |= (1<<6);
	prevMotorEnable = 0;
	
	// set pulse states low
	lowPulseState = 0;
	highPulseState = 0;
	
	//reset running var's
	cycleTime = 0;
	halfTime = 0;
	hertzPulse = 0.0;
	rpmPulse = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// handle the motor enable/disable function
		motorEnableSafe();
		
		// check for RX data
		if(rxDataAvailable == 1){
			rxDataProcessing();
		}
		
		// check if the motor is enabled
		if(motorEnable == 1){	
			
			// handle motor cold start
			motorStartup();
			
			// handle stable rpm feedback
			rpmShifter();
			
			// check if pid is enabled
			if((pidEnable == 1) && (motorRunning == 1)){
				// do PID calculations
				PIDcalculations();
							
				// ramp the motor rpm setpoint
				RPMmotorRamp();
			}
			
		}
		// get and shift analog inputs
		analogShifter();
		
		// use dataselector to get the correct setpoints
		dataSelector();
		
		// do power calculations
		powerCalculations();
		
		// do motor RPM calculations and motor run parameters
		motorCalculations();
			
		// determine and control motor state
		motorRunningState();
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 5;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
	HAL_ADC_Start_DMA(&hadc3, analogInputs, 5);
  /* USER CODE END ADC3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 180-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

	// initialize one shot timer
	TIM_CCxChannelCmd(htim3.Instance, TIM_CHANNEL_3,TIM_CCx_ENABLE);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 180-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

	// initialize one shot timer
	TIM_CCxChannelCmd(htim4.Instance, TIM_CHANNEL_1,TIM_CCx_ENABLE);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
	
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 90-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motorEnableOut_GPIO_Port, motorEnableOut_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : High_Side_Pole_Pin Low_Side_Pole_Pin */
  GPIO_InitStruct.Pin = High_Side_Pole_Pin|Low_Side_Pole_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : motorEnable_Pin */
  GPIO_InitStruct.Pin = motorEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(motorEnable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : motorEnableOut_Pin */
  GPIO_InitStruct.Pin = motorEnableOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motorEnableOut_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//Do something while the analog conversion has finished??
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	//Do something while the uart recieve has been triggered??	
	rxDataAvailable = 1;
	
	__HAL_UART_CLEAR_OREFLAG(huart);
	huart->ErrorCode |= HAL_UART_ERROR_ORE;
}
		
void motorEnableSafe(void){
	// enable/disable the motor on a rising/falling trigger
	if((motorEnable == 1) && (prevMotorEnable == 0)){
		GPIOC->BSRR |= (1<<6) <<16;
		pulseWidth = 500;
		TIM5->CNT = 0;
		prevMotorEnable = 1;
		rpmActSetPulse = ((rpmSetPulse - minimumMotorFreq) / 3) + minimumMotorFreq;
	}
	else if((motorEnable == 0) && (prevMotorEnable == 1)){
		GPIOC->BSRR |= (1<<6);
		prevMotorEnable = 0;
		lowPulseState = 0;
		highPulseState = 0;
		cycleTime = 0;
		halfTime = 0;
		hertzPulse = 0.0;
		rpmPulse = 0;
	}
	
	// if the motor is off reset all variable of the PID
	if((pidEnable == 0) || (motorEnable == 0)){
		error = 0.0;
		cumError = 0.0;
		rateError = 0.0;
		lastError = 0.0;
		multiplierPid = 1000.0;
	}
}

void PIDcalculations(void){	
	if(((highPulseState == 1) || (lowPulseState == 1))){
		if(pidAvgInput == 1){
			error = (rpmActSetPulse - rpmAvg); // determine Proportional
		}
		else{
			error = (rpmActSetPulse - rpmPulse); // determine Proportional
		}
		cumError += error; // determine Integral
		rateError = (error - lastError); // determine Derivative
		multiplierPid = kp*error + ki*cumError + kd*rateError; // sum them together 
		lastError = error;  // record error for next cycle
	
		// catch duty cycle overflow
		if(multiplierPid > PID_MAX){
			multiplierPid = PID_MAX;
		}
		else if(multiplierPid < PID_MIN){
			multiplierPid = PID_MIN;
		}
	}
}

void RPMmotorRamp(void){
	// setpoint change
	if((rpmSetPulse != rpmSetPulsePrev)){
		rpmSetPulsePrev = rpmSetPulse;
		motorAtSpeed = 0;
			
		// calculate new incremental
		if(rpmSetPulse > rpmActSetPulse){
			rpmPulseIncremental = (rpmSetPulse - rpmActSetPulse) / rpmPulseAttackTime;
		}
		else if(rpmSetPulse < rpmActSetPulse){
			rpmPulseIncremental = (rpmActSetPulse - rpmSetPulse) / rpmPulseAttackTime;
		}
	}
		
	// do the actual rampup to the setpoint
	if((TIM5->CNT >= 10) && (incrementalPulseDone == 0)){
		incrementalPulseDone = 1; // set the done bit for one pulse mode
		// ramp up/down for the rpm setpoint
		if((rpmSetPulse > (rpmActSetPulse * 1.01)) && (motorAtSpeed == 0)){
			rpmActSetPulse += rpmPulseIncremental;
		}
		else if((rpmSetPulse < (rpmActSetPulse * 0.99)) && (motorAtSpeed == 0)){
			rpmActSetPulse -= rpmPulseIncremental;
		}
		else{
			rpmActSetPulse = rpmSetPulse;
		}
		
		if((rpmPulse >= (rpmActSetPulse * 0.99)) && (rpmPulse <= (rpmActSetPulse * 1.01))){
			motorAtSpeed = 1;
		}
		else{
			motorAtSpeed =0;
		}
	}
}

void motorStartup(void){
	// check if the motor is not running running yet
	if((motorRunning == 0)){
		// set startup pulses
		
		// set startup parameters
		if(motorStartupState == 0){
			TIM3->CCR3 = 1;
			TIM3->ARR = 65534;
			TIM4->CCR1 = 1;
			TIM4->ARR = 65534;
			motorStartupState = 1;
		}
		// start first pulse
		else if(motorStartupState == 1){
			TIM5->CNT = 0;
			if(pulseCycle == 0){
				__HAL_TIM_ENABLE(&htim3); // enable one shot pwm timer
				pulseCycle = 1;
			}
			else{
				__HAL_TIM_ENABLE(&htim4); // enable one shot pwm timer
				pulseCycle = 0;
			}
			motorStartupState = 2;
		}
		// control ramp of of frequency
		else if(motorStartupState == 2){
			// control lowside pulse
			if((pulseCycle == 1) && (TIM5->CNT >= (startupFrequency / 2))){
				pulseCycle = 0;
				__HAL_TIM_ENABLE(&htim4); // enable one shot pwm timer
				if(startupFrequency > startupFrequencyDecr){
					startupFrequency -= startupFrequencyDecr;
				}
			}
			// control highside pulse
			else if((pulseCycle == 0) && (TIM5->CNT >= startupFrequency)){
				TIM5->CNT = 0;
				__HAL_TIM_ENABLE(&htim3); // enable one shot pwm timer
				if(startupFrequency > startupFrequencyDecr){
					startupFrequency -= startupFrequencyDecr;
				}
			}
			
			// if not running restart
			if((startupFrequency < startupFrequencyDecr) && (motorRunning == 0)){
				startupFrequency = startupSetpoint;
			}
		}
		
	}
	// if running reset the startup ramp
	else if(motorRunning == 1){
		startupFrequency = startupSetpoint;
		motorStartupState = 0;
	}
}

void motorRunningState(void){
	// figure if the motor is above minimum hertz
	if((hertzPulse > minimumMotorFreq) && (motorEnable == 1)){
		motorRunning = 1;
		if(pidEnable == 1){
			widthSetpoint = 1000;
		}
	}
	else if(motorEnable == 0){
		motorRunning = 0;
	}
	
	// reset pulse states
	if(highPulseState == 1){
		incrementalPulseDone = 0; // reset incremental
		highPulseState = 0;
	}
	if(lowPulseState == 1){
		incrementalPulseDone = 0; // reset incremental
		lowPulseState = 0;
	}
	
	// if motor disabled reset ramp setpoint
	if(motorEnable == 0){
		rpmActSetPulse = 0;
	}
}

void rpmShifter(void){
	rpmShift[0] = rpmPulse;
	
	for(int i = 0; i <= RPM_SHIFT; i++){
		rpmShift[i] = rpmShift[(i-1)];
	}
	// reset rpm average
	rpmAvgCalc = 0;		
	
	// add all rpm registers
	for(int i = 0; i < (RPM_SHIFT); i++){
		rpmAvgCalc += rpmShift[i];
	}
					
	// divide by the registers	
	if(rpmAvgCalc > 0){
		rpmAvg = rpmAvgCalc / RPM_SHIFT;
	}
	
	if(cycleBufferEnable == 1){
		cycleShift[0][0] = halfTimeRaw;
		cycleShift[0][1] = cycleTimeRaw;
	
		for(int i = 0; i <= CYCLETIME_SHIFT; i++){
			cycleShift[i][0] = cycleShift[(i-1)][0];
			cycleShift[i][1] = cycleShift[(i-1)][1];
		}
		// reset rpm average
		cycleAvgCalc = 0;	
		halfCycleAvgCalc = 0;		
	
		// add all rpm registers
		for(int i = 0; i < (CYCLETIME_SHIFT); i++){
			halfCycleAvgCalc += cycleShift[i][0];
			cycleAvgCalc += cycleShift[i][1];
		}
					
		// divide by the registers	
		if(halfCycleAvgCalc > 0){
			halfTime = halfCycleAvgCalc / CYCLETIME_SHIFT;
		}
		if(cycleAvgCalc > 0){
			cycleTime = cycleAvgCalc / CYCLETIME_SHIFT;
		}
	}
	else{
		halfTime = halfTimeRaw;
		cycleTime = cycleTimeRaw;
	}
}

void analogShifter(void){
	// shift analog input array
	for(int i = (SHIFT_ARRAY - 1);i >= 0;i--){
		for(int ii = 0;ii <= 5;ii++){
			analogShift[i][ii] = analogShift[(i-1)][ii];
		}
	}
	
	// reset voltage average
	voltageAvgCalc = 0;
			
	// add all voltage registers
	for(int i = 0; i < (VOLTAGE_SHIFT); i++){
		voltageAvgCalc += analogShift[i][0];
	}
					
	// divide by the registers	
	if(voltageAvgCalc > 0){
		voltageAvg = voltageAvgCalc / VOLTAGE_SHIFT;
	}
	
	// reset current average
	currentAvgCalc = 0;
			
	// add all current registers
	for(int i = 0; i < (CURRENT_SHIFT); i++){
		currentAvgCalc += analogShift[i][1];
	}
					
	// divide by the registers	
	if(currentAvgCalc > 0){
		currentAvg = currentAvgCalc / CURRENT_SHIFT;
	}
	
	// reset temperature average
	tempAvgCalc = 0;
			
	// add all temperature registers
	for(int i = 0; i < (TEMP_SHIFT); i++){
		tempAvgCalc += analogShift[i][1];
	}
					
	// divide by the registers	
	if(currentAvgCalc > 0){
		tempAvg = tempAvgCalc / TEMP_SHIFT;
	}
	
	// reset delay average
	pulseDelayAvgCalc = 0;
			
	// add all delay registers
	for(int i = 0; i < (DELAY_SHIFT); i++){
		pulseDelayAvgCalc += analogShift[i][2];
	}
						
	// divide by the registers	
	if(pulseDelayAvgCalc > 0){
		pulseDelayAvg = pulseDelayAvgCalc / DELAY_SHIFT;
	}
		
	// reset width average
	pulseWidthAvgCalc = 0;
			
	// add all width registers
	for(int i = 0; i < (WIDTH_SHIFT); i++){
		pulseWidthAvgCalc += analogShift[i][3];
	}
			
	// divide by the registers		
	if(pulseWidthAvgCalc > 0){
		pulseWidthAvg = pulseWidthAvgCalc / WIDTH_SHIFT;
	}
}

void dataSelector(void){
	if(uartEnable == 0){ // get analog values
		// motorEnable == digital input
		delaySetpoint = ((1000.0 / 4096.0) * (float) pulseDelayAvg);
		widthSetpoint = ((1000.0 / 4096.0) * (float) pulseWidthAvg);
		rpmSetPulse = ((maxRpmSetpoint / 4096.0) * (float) pulseWidthAvg);
	}
	else if(uartEnable == 1){ // get uart values
		motorEnable == serialMotorEnable;
		delaySetpoint = (float)serialDelaySetpoint;
		widthSetpoint = (float)serialWidthSetpoint;
		rpmSetPulse = serialRpmSetpoint;
	}
}

void powerCalculations(void){
	motorVoltage = voltageAvg * voltageMultiplier;
	motorCurrent = currentAvg * currentMultiplier;
	motorTemperature = tempAvg * tempMultiplier;
}

void motorCalculations(void){
	if(cycleTime > 0){ // catch divide by 0
		hertzPulse = (1000000.0 / (float) cycleTime); // calculate motor coil hertz
		rpmPulse = (hertzPulse / uniPolePass)  * 60.0; // calculate rpm
		pulseDelay = (((((float) cycleTime / 2.0) / 1000.0) * (float) delaySetpoint)); // calculate pulse time delay
		pulseWidth = (uint32_t) (((((((float) cycleTime / 2.0) / 1000.0) * (float) widthSetpoint) / 2.0) / 1000.0) * (float)multiplierPid); // calculate pulse width
			
		if((pulseWidth + pulseDelay) > 65535){ // catch overflow
			pulseWidth = 65534 - pulseDelay;
		}
	}
	else{ // set pulse hz and rpm to 0
		rpmPulse = 0.0;
		hertzPulse = 0.0;
	}			
}

void rxDataProcessing(){
	int start = 0;
	for(int i = 0; i <= 15; i++){
		if ((Rx_data[i] == 0x80) || (Rx_data[i] == 0x70)){
			start = i;
		}
	}
	
	// if the uart command was a register write action
	if((Rx_data[start] == 0x80) && (start < 9)){
		switch(Rx_data[(start + 1)]){
			case 0x11: // motor enable
				break;
			case 0x12: // pid enable
				break;
			case 0x13: // uart enable
				break;
			case 0x14: // cycle avg enable
				break;
			case 0x21: // motor running
				break;
			case 0x22: // motor at speed
				break;
			case 0x31: // cycle time
				break;
			case 0x32: // half time
				break;
			case 0x33: // motor hertz
				break;
			case 0x34: // motor rpm
				break;
			case 0x35: // motor rpm avg
				break;
			case 0x36: // actual pid setpoint
				break;
			case 0x41: // delay setpoint
				break;
			case 0x42: // width setpoint
				break;
			case 0x43: // rpm setpoint
				break;
			case 0x44: // ramp up/down time
				break;
			case 0x51: // startup frequency
				break;
			case 0x52: // sweep speed 
				break;
			case 0x53: // minimum speed 
				break;
			case 0x61: // bus voltage
				break;
			case 0x62: // motor current
				break;
			case 0x63: // temperature
				break;
			case 0x71: // pid avg input
				break;
			case 0x72: // pid minimum
				break;
			case 0x73: // pid maximum
				break;
			case 0x74: // Proportional
				break;
			case 0x75: // Integral
				break;
			case 0x76: // Derivative
				break;		
		}
	}
	else if((Rx_data[start] == 0x70) && (start < 9)){
		switch(Rx_data[(start + 1)]){
			case 0x11: // motor enable
				break;
			case 0x12: // pid enable
				break;
			case 0x13: // uart enable
				break;
			case 0x14: // cycle avg enable
				break;
			case 0x21: // motor running
				break;
			case 0x22: // motor at speed
				break;
			case 0x31: // cycle time
				break;
			case 0x32: // half time
				break;
			case 0x33: // motor hertz
				break;
			case 0x34: // motor rpm
				break;
			case 0x35: // motor rpm avg
				break;
			case 0x36: // actual pid setpoint
				break;
			case 0x41: // delay setpoint
				break;
			case 0x42: // width setpoint
				break;
			case 0x43: // rpm setpoint
				break;
			case 0x44: // ramp up/down time
				break;
			case 0x51: // startup frequency
				break;
			case 0x52: // sweep speed 
				break;
			case 0x53: // minimum speed 
				break;
			case 0x61: // bus voltage
				break;
			case 0x62: // motor current
				break;
			case 0x63: // temperature
				break;
			case 0x71: // pid avg input
				break;
			case 0x72: // pid minimum
				break;
			case 0x73: // pid maximum
				break;
			case 0x74: // Proportional
				break;
			case 0x75: // Integral
				break;
			case 0x76: // Derivative
				break;		
		}
	}
			
//				case 0x01: // enable bus control
//					uartEnable = Rx_data[(start + 5)]; // 0=analog 1=busvalue
//					data[start]= 0x80;
//					data[1]= 0x01;
//					data[2]=0x00;
//					data[3]=0x00;
//					data[4]=0x00;
//					data[5]=(uartEnable);
//				data[6]=0x11;
//				data[7]=0x12;
//					HAL_UART_Transmit(&huart5,data,sizeof(data),10);
//					break;
	
		

//				case 0x05: // set rpm
//					serialRpmSetpoint = (Rx_data[(start + 2)] << 24) | (Rx_data[(start + 3)] << 16) | (Rx_data[(start + 4)] << 8) | (Rx_data[(start + 5)]);
//					data[0]= 0x80;
//					data[1]= 0x05;
//					data[2]=(serialRpmSetpoint >> 24);
//					data[3]=(serialRpmSetpoint >> 16);
//					data[4]=(serialRpmSetpoint >> 8);
//					data[5]=(serialRpmSetpoint);
//				data[6]=0x11;
//				data[7]=0x12;
//					HAL_UART_Transmit(&huart5,data,sizeof(data),10);
//					break;

			
//				case 0x64: // set pid max tune
//					data[0]= 0x80;
//					data[1]= 0x64;
//					data[2]=(PID_MAX >> 24);
//					data[3]=(PID_MAX >> 16);
//					data[4]=(PID_MAX >> 8);
//					data[5]=(PID_MAX);
//					data[6]=0x11;
//					data[7]=0x12;
//					HAL_UART_Transmit(&huart5,data,sizeof(data),10);
//					break;
					
//								case 4:
				//Send motor data over uart
				// calculate pulse delay
//				data[0]= 0x70;
//				data[1]= 0x50;
//				data[2]=(pulseDelay >> 24);
//				data[3]=(pulseDelay >> 16);
//				data[4]=(pulseDelay >> 8);
//				data[5]=(pulseDelay);
//				data[6]=0x11;
//				data[7]=0x12;
//				HAL_UART_Transmit(&huart5,data,sizeof(data),10);
//				break;

			
	// set all databits to 0
	for(int i = 0; i <= 15; i++){
		Rx_data[i] = 0x00;
	}
		
	// reset the data command flag
	rxDataAvailable = 0;
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
