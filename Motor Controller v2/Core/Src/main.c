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

/* USER CODE BEGIN PV */

uint32_t cycleTime;
uint32_t halfTime;

uint32_t pulseDelay = 0;
uint32_t pulseWidth = 1000;

int highPulseState;
int lowPulseState;

float uniPolePass = 3.0;
float rpmPulse;
float hertzPulse;

uint32_t delaySetpoint = 0;
uint32_t widthSetpoint = 1000;

int motorEnable;
int motorRunning;
int prevMotorEnable;

float rpmSetPulse = 500;
float rpmSetPulsePrev;
float rpmActSetPulse = 500;
float rpmPulseIncremental;
float rpmPulseAttackTime = 100;
float minimumMotorFreq = 2.0;

uint32_t startupFrequency = 500000;

float error;
float cumError;
float rateError;
float lastError; 
float multiplierPid;
float kp = 3.0;
float ki = 0.00045;
float kd = 0.01;  
float PID_MIN = 1.0;
float PID_MAX = 1000.0;

int incrementalPulseDone;
int motorAtSpeed;

int pidEnable = 1;
int pulseCycle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

static void pulseControl(void);
static void PIDcalculations(void);
static void RPMmotorRamp(void);
static void motorStartup(void);
static void motorRunningState(void);
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
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim5);
		GPIOC->BSRR |= (1<<6);
		//GPIOC->BSRR |= (1<<6) <<16;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// handle the motor enable/disable function
		motorEnableSafe();
		
		// check if the motor is enabled
		if(motorEnable == 1){
			
			// handle the motor pulses
			pulseControl();		
			// handle motor cold start
			//motorStartup();
			
			// check if pid is enabled
			if((pidEnable == 1)){
				// do PID calculations
				PIDcalculations();
			}
			
			// ramp the motor rpm setpoint
			RPMmotorRamp();
		}
		
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
  hadc3.Init.NbrOfConversion = 4;
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
  /* USER CODE BEGIN ADC3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motorEnable_GPIO_Port, motorEnable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : High_Side_Pole_Pin Low_Side_Pole_Pin */
  GPIO_InitStruct.Pin = High_Side_Pole_Pin|Low_Side_Pole_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : motorEnable_Pin */
  GPIO_InitStruct.Pin = motorEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motorEnable_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void pulseControl(){
	// handle the highside pulse
	if((highPulseState == 1) && (TIM5->CNT >= (pulseDelay)) && (pulseCycle == 0)){
		incrementalPulseDone = 0; // reset incremental
		highPulseState = 2; // set state int
		pulseCycle = 1;
		TIM3->ARR = pulseWidth; // set pulsewidth
		__HAL_TIM_ENABLE(&htim3); // enable one shot pwm timer
	}
	
	// handle the lowside pulse
	if((lowPulseState == 1) && (TIM5->CNT >= (pulseDelay + halfTime)) && (pulseCycle == 1)){
		incrementalPulseDone = 0; // reset incremental bit
		lowPulseState = 2; // set state int
		pulseCycle = 0;
		TIM4->ARR = pulseWidth; // set pulsewidth
		__HAL_TIM_ENABLE(&htim4); // enable one shot pwm timer
	}
	
	if((lowPulseState == 1) && (highPulseState == 1) && (motorRunning == 0)){
		lowPulseState = 0;
		highPulseState = 0;
	}
}
		
void motorEnableSafe(void){
	// enable/disable the motor on a rising/falling trigger
	if((motorEnable == 1) && (prevMotorEnable == 0)){
		GPIOC->BSRR |= (1<<6) <<16;
		pulseWidth = 500;
		TIM5->CNT = 0;
		prevMotorEnable = 1;
		rpmActSetPulse = rpmSetPulse / 3;
	}
	else if((motorEnable == 0) && (prevMotorEnable == 1)){
		GPIOC->BSRR |= (1<<6);
		prevMotorEnable = 0;
		lowPulseState = 0;
		highPulseState = 0;
		cycleTime = 0;
		halfTime = 0;
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
	if((motorRunning == 1) && ((highPulseState == 2) || (lowPulseState == 2))){
		error = (rpmActSetPulse - rpmPulse); // determine Proportional
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
		if((rpmSetPulse > (rpmActSetPulse * 1.02)) && (motorAtSpeed == 0)){
			rpmActSetPulse += rpmPulseIncremental;
		}
		else if((rpmSetPulse < (rpmActSetPulse * 0.98)) && (motorAtSpeed == 0)){
			rpmActSetPulse -= rpmPulseIncremental;
		}
		else{
			rpmActSetPulse = rpmSetPulse;
			motorAtSpeed = 1;
		}
	}
}

void motorStartup(void){
	// check if the motor is not running running yet
	if((motorRunning == 0)){
		// set startup pulses
		if((TIM5->CNT > 1) && (highPulseState == 0) && (lowPulseState != 1)){
			cycleTime = startupFrequency;
			highPulseState = 1;
			halfTime = startupFrequency / 2.0;
		}
		
		if((highPulseState == 2)){
			lowPulseState = 1;
		}
		
		// reset timer 
		if(TIM5->CNT > startupFrequency){
			TIM5->CNT = 0;
		}
	}
}

void motorRunningState(void){
	// figure if the motor is above minimum hertz
	if((hertzPulse > minimumMotorFreq)){
		motorRunning = 1;
	}
	else if((hertzPulse > minimumMotorFreq) && (pidEnable == 1)){
		motorRunning = 1;
		pulseWidth = 1000;
	}
	else{
		motorRunning = 0;
	}
	
	// reset pulse states
	if(highPulseState == 2){
		highPulseState = 0;
	}
	if(lowPulseState == 2){
		lowPulseState = 0;
	}
	
	// if motor disabled reset ramp setpoint
	if(motorEnable == 0){
		rpmActSetPulse = 0;
	}
}

void motorCalculations(void){
	if(((highPulseState == 0) && (lowPulseState == 0)) || ((highPulseState == 2) && (lowPulseState == 0)) || ((highPulseState == 0) && (lowPulseState == 2))){ // if no pulse is running
		if(cycleTime > 0){ // catch divide by 0
			hertzPulse = (1000000.0 / (float) cycleTime); // calculate motor coil hertz
			rpmPulse = (hertzPulse / uniPolePass)  * 60.0; // calculate rpm
			pulseDelay = (((((float) cycleTime / 2.0) / 1000.0) * (float) delaySetpoint)); // calculate pulse time delay
			pulseWidth = (uint32_t) (((((((float) cycleTime / 2.0) / 1000.0) * (float) widthSetpoint) / 2.0) / 1000.0) * (float)multiplierPid); // calculate pulse width
			
			if(pulseWidth > 65535){ // catch overflow
				pulseWidth = 65534;
			}
		}
		else{ // set pulse hz and rpm to 0
			rpmPulse = 0.0;
			hertzPulse = 0.0;
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
