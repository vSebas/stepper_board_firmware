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
#include <math.h>
#include "stdio.h"
#include "stdlib.h"
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
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for canTxTask */
osThreadId_t canTxTaskHandle;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for steeringTaskPWM */
osThreadId_t steeringTaskPWMHandle;
const osThreadAttr_t steeringTaskPWM_attributes = {
  .name = "steeringTaskPWM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal4,
};
/* Definitions for periodicTimer */
osTimerId_t periodicTimerHandle;
const osTimerAttr_t periodicTimer_attributes = {
  .name = "periodicTimer"
};
/* Definitions for canRxSem */
osSemaphoreId_t canRxSemHandle;
const osSemaphoreAttr_t canRxSem_attributes = {
  .name = "canRxSem"
};
/* USER CODE BEGIN PV */
const float MAX_STEERING = 57.3; // Degrees
const float STEP_ANGLE = 1.8; // Degrees
const float STEPS_REV = 200;  // Steps per revolution
int dir_1 = 1;				 //
int goal_steps_1 = 0;		 // Required steps to reach desired angle
int steps_1 = 0;			  // Steps of motor 1
int current_step_1 = 0;
float desired_angle_1 = 45; // Degrees
float current_angle_1 = 0; // Degrees
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
void can_rx_task(void *argument);
void can_tx_task(void *argument);
void led_task(void *argument);
void steering_task_pwm(void *argument);
void PeriodicTimerCallback(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	for(int i=0; i<len; i++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}

uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
uint32_t FreeMailBoxes;

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;

//uint8_t count = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//osSemaphoreRelease(canRxSemHandle);
	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		  HAL_GPIO_TogglePin(GPIOA, DEBUG_2_Pin);
	  osDelay(50);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  // Debug
  HAL_GPIO_WritePin(GPIOC, DEBUG_6_Pin|DEBUG_5_Pin|DEBUG_4_Pin|DEBUG_3_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOA, DEBUG_2_Pin|DEBUG_1_Pin, GPIO_PIN_SET);

  // Stepper 1
  HAL_GPIO_WritePin(LVL_SFTR_OE_1_GPIO_Port, LVL_SFTR_OE_1_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOC, STPR_DIR_1_Pin | STPR_EN_1_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(STPR_PWM_1_GPIO_Port, STPR_PWM_1_Pin, GPIO_PIN_RESET);

  // Stepper 2
  //HAL_GPIO_WritePin(GPIOB, LVL_SFTR_OE_2_Pin|STPR_DIR_2_Pin|STPR_PWM_2_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOB, STPR_EN_2_Pin, GPIO_PIN_SET);

  // ADC Calibration
  /*
  HAL_StatusTypeDef ret;
  ret = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if(ret != HAL_OK)
  {
	  Error_Handler();
  }

*/

  TIM1->CCR1 = 50;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  if(HAL_CAN_Start(&hcan1) == HAL_OK)
	  printf("CAN started correctly\n");
  else
	  printf("CAN error\n");

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of canRxSem */
  canRxSemHandle = osSemaphoreNew(1, 1, &canRxSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of periodicTimer */
  periodicTimerHandle = osTimerNew(PeriodicTimerCallback, osTimerPeriodic, NULL, &periodicTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(can_rx_task, NULL, &canRxTask_attributes);

  /* creation of canTxTask */
  canTxTaskHandle = osThreadNew(can_tx_task, NULL, &canTxTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(led_task, NULL, &ledTask_attributes);

  /* creation of steeringTaskPWM */
  steeringTaskPWMHandle = osThreadNew(steering_task_pwm, NULL, &steeringTaskPWM_attributes);

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
  /*
  uint32_t previous_t = 0;
  uint32_t current_t = 0;
  uint32_t delay = 1;
  uint32_t thermistor_reading = 0;

  GPIO_PinState ID_0;
  GPIO_PinState ID_1;
  GPIO_PinState ID_2;
  GPIO_PinState ID_3;

  GPIO_PinState BRAKE;
  GPIO_PinState FAULT_1;
  GPIO_PinState FAULT_2;
  */

  //HAL_UART_Transmit(&huart1, dataT, 14, HAL_MAX_DELAY);

  while (1)
  {
	  /*
	  current_t = HAL_GetTick();

	  if(current_t - previous_t >= delay){
		  previous_t = current_t;
		  HAL_GPIO_WritePin(STPR_PWM_1_GPIO_Port, STPR_PWM_1_Pin, GPIO_PIN_SET);

		  HAL_Delay(delay);

		  HAL_GPIO_WritePin(STPR_PWM_1_GPIO_Port, STPR_PWM_1_Pin, GPIO_PIN_RESET);
	  }

	  ID_0 = HAL_GPIO_ReadPin(GPIOB, ID_0_Pin);
	  ID_1 = HAL_GPIO_ReadPin(GPIOB, ID_1_Pin);
	  ID_2 = HAL_GPIO_ReadPin(GPIOB, ID_2_Pin);
	  ID_3 = HAL_GPIO_ReadPin(GPIOB, ID_3_Pin);

	  BRAKE = HAL_GPIO_ReadPin(GPIOB, BRAKE_IN_Pin);
	  FAULT_1 = HAL_GPIO_ReadPin(GPIOC, STPR_FLT_1_Pin);
	  FAULT_2 = HAL_GPIO_ReadPin(GPIOB, STPR_FLT_2_Pin);

	  printf("ID 0: %d\n", ID_0);
	  printf("ID 1: %d\n", ID_1);
	  printf("ID 2: %d\n", ID_2);
	  printf("ID 3: %d\n", ID_3);
	  printf("\n");
	  printf("BRAKE: %d\n", BRAKE);
	  printf("FAULT 1: %d\n", FAULT_1);
	  printf("FAULT 2: %d\n", FAULT_2);

	  HAL_ADC_Start(&hadc1);
	  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  {
		  thermistor_reading = HAL_ADC_GetValue(&hadc1);
	  }
	  HAL_ADC_Stop(&hadc1);

	  printf("Thermistor Measurement: %u\n", thermistor_reading);
	  printf("Thermistor Voltage: %f\n", (float) thermistor_reading*3.3/4096);
		*/

	  //if(HAL_UART_Receive(&huart1, dataR, 1, HAL_MAX_DELAY) == HAL_OK)
	  //{
	  // dataR[0] = dataR[0] + 1;
	  // HAL_UART_Transmit(&huart1, dataR, 1, HAL_MAX_DELAY);
	  // }

	  //if(!HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)){
		  //printf("Message sent 2\n");
		//  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[0], &TxMailbox);
			  /* printf("Transmission requested 2\n");
		  else
			  printf("Error 2\n");
	  //}
	  else
		  printf("Message pending 2\n");*/

	  //HAL_Delay(500);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 40;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;		// Specify filter bank to use
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //Incoming data is saved here
  canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh= 0;
  canfilterconfig.FilterMaskIdLow = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1 ;
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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DEBUG_6_Pin|DEBUG_5_Pin|DEBUG_4_Pin|DEBUG_3_Pin
                          |STPR_EN_1_Pin|STPR_DIR_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DEBUG_2_Pin|DEBUG_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LVL_SFTR_OE_2_Pin|STPR_DIR_2_Pin|STPR_EN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LVL_SFTR_OE_1_GPIO_Port, LVL_SFTR_OE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DEBUG_6_Pin DEBUG_5_Pin DEBUG_4_Pin DEBUG_3_Pin
                           STPR_EN_1_Pin STPR_DIR_1_Pin */
  GPIO_InitStruct.Pin = DEBUG_6_Pin|DEBUG_5_Pin|DEBUG_4_Pin|DEBUG_3_Pin
                          |STPR_EN_1_Pin|STPR_DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DEBUG_2_Pin DEBUG_1_Pin */
  GPIO_InitStruct.Pin = DEBUG_2_Pin|DEBUG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LVL_SFTR_OE_2_Pin STPR_DIR_2_Pin STPR_EN_2_Pin */
  GPIO_InitStruct.Pin = LVL_SFTR_OE_2_Pin|STPR_DIR_2_Pin|STPR_EN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STPR_FLT_2_Pin ID_0_Pin ID_1_Pin ID_2_Pin
                           ID_3_Pin BRAKE_IN_Pin */
  GPIO_InitStruct.Pin = STPR_FLT_2_Pin|ID_0_Pin|ID_1_Pin|ID_2_Pin
                          |ID_3_Pin|BRAKE_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E_STOP_Pin STPR_FLT_1_Pin */
  GPIO_InitStruct.Pin = E_STOP_Pin|STPR_FLT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LVL_SFTR_OE_1_Pin */
  GPIO_InitStruct.Pin = LVL_SFTR_OE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LVL_SFTR_OE_1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_can_rx_task */
/**
  * @brief  Function implementing the canRxTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_can_rx_task */
void can_rx_task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  //osThreadSuspend(canRxTaskHandle);
	  osSemaphoreAcquire(canRxSemHandle, osWaitForever);
	  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		  HAL_GPIO_TogglePin(GPIOA, DEBUG_2_Pin);
	  osDelay(50);
  }
  //osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_can_tx_task */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_tx_task */
void can_tx_task(void *argument)
{
  /* USER CODE BEGIN can_tx_task */
  /* Infinite loop */
  for(;;)
  {
	  TxHeader.DLC = 1; // Length of data to send in bytes
		TxHeader.ExtId = 0; // For basic CAN protocol
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA; // transfering data of remote frame
		TxHeader.StdId = 0x600; // ID of this CAN peripheral
		TxHeader.TransmitGlobalTime = DISABLE;

		TxData[0] = 0x11;

		FreeMailBoxes = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);

		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[0], &TxMailbox) == HAL_OK) {
		  printf("Transmission requested 1\n");
				  if(!HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
					  printf("Message sent 1\n");
				  else
					  printf("Message pending 1\n");
			  } else
				  printf("Error 1\n");
		osDelay(50);
  }
  osThreadTerminate(NULL);
  /* USER CODE END can_tx_task */
}

/* USER CODE BEGIN Header_led_task */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_task */
void led_task(void *argument)
{
  /* USER CODE BEGIN led_task */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, DEBUG_1_Pin);
	osDelay(600);
  }
  osThreadTerminate(NULL);
  /* USER CODE END led_task */
}

/* USER CODE BEGIN Header_steering_task_pwm */
/**
* @brief Function implementing the steeringTaskPWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_steering_task_pwm */
void steering_task_pwm(void *argument)
{
  /* USER CODE BEGIN steering_task_pwm */
  /* Infinite loop */
  for(;;)
  {
		int angle_d = (int) desired_angle_1;
		dir_1 = abs(angle_d)/angle_d;

		if(fabs(desired_angle_1) > MAX_STEERING)
			desired_angle_1 = MAX_STEERING;

		desired_angle_1 *= dir_1; // to work only with positive numbers
		goal_steps_1 = desired_angle_1/STEP_ANGLE;

		HAL_GPIO_WritePin(GPIOC, STPR_DIR_1_Pin | STPR_EN_1_Pin, dir_1);

		if(steps_1 < goal_steps_1)
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		else
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  }
  /* USER CODE END steering_task_pwm */
}

/* PeriodicTimerCallback function */
void PeriodicTimerCallback(void *argument)
{
  /* USER CODE BEGIN PeriodicTimerCallback */
	HAL_GPIO_TogglePin(GPIOA, DEBUG_1_Pin);
  /* USER CODE END PeriodicTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
