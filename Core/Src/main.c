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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HandCotrol.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "stm32f4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



extern uint8_t ADC_Data[500];
extern uint32_t drts;
extern uint32_t dstc;
extern EncoderSens Encoder[6];
extern MotorDefinition Motor[6];
extern uint32_t glb_dstc;
extern double Coef_P;
extern double Coef_I;
extern double Coef_T;
extern uint16_t ContRegulatorValue;

extern uint8_t UsartDataByte;
extern uint8_t UsartData[40];
extern uint32_t UsartDataCnt;
extern uint32_t UsartDataCnt2;

extern _Bool ThisDeviceOnUsartCtrl;
extern _Bool ETEMode_Enable;

uint32_t EncCnt[6];
uint32_t EncCntNow[6];
uint32_t EncCntOld[6];
uint32_t d_EncCnt[6];

uint32_t EncTime[6];
uint32_t EncTimeNow[6];
uint32_t EncTimeOld[6];

uint32_t d_EncCntOld[6];
uint32_t d_EncTime[6][500];
double d_Velocity[6][500];


double RegVal[1000];

uint16_t SpeedAngleMas[5000];


uint32_t LimitCNT;
_Bool flag_motor_is_move = false;

extern _Bool DeviceIsConnected;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM11_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  PR_TIM11_ON;
  PR_TIM10_ON;
  /*Init*/
  ProtezInit();

  HAL_UART_Receive_IT(&huart6, &UsartDataByte, 1);

  HAL_Delay(1000);
//  PR_TIM11_ON;
  uint8_t dd[3] = {0xDD, 0xDD, 0xDD};
  uint32_t gintsts = USB_OTG_FS->GINTSTS;

  //StartMeasurement();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_PCD_GetConnectionState(&hpcd_USB_OTG_FS) == USB_CONNECTED

//	  if(!DeviceIsConnected)
//	  {
//		  CDC_Transmit_FS(dd, 3);
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		  HAL_Delay(100);
//	  }


//		// Проверяем на событие сброса USB
//		if (gintsts & USB_OTG_GINTSTS_USBRST)
//		{
//			MX_USB_DEVICE_Init();
//		}
//		// Проверяем на завершение перечисления
//		if (gintsts & USB_OTG_GINTSTS_ENUMDNE)
//		{
//			// Устройство успешно подключено и перечислено
//			// Здесь можно начать обмен данными
//		}
//		// Проверяем на начало кадра
//		if (gintsts & USB_OTG_GINTSTS_SOF)
//		{
//			// Обработка получения кадра начала кадра
//			// Это может быть полезно для синхронизации
//
//		}
//		// тип устройство подключено
//		if (gintsts & USB_OTG_GINTSTS_IEPINT)
//		{
//			DeviceIsConnected = true;
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			HAL_Delay(50);
//		}
//		// Сброс флагов (если необходимо)
//		USB_OTG_FS->GINTSTS = gintsts; // Сбрасываем обработанные флаги



	  // USBD_CDC.DISCINT
	  //USB_OTG_GINTSTS_ENUMDNE
	  //USB_OTG_GINTSTS_SOF
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

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
  htim4.Init.Prescaler = 1000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 60-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 100-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 6000;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 100;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DR_RW_GPIO_Port, DR_RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_Pin */
  GPIO_InitStruct.Pin = BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_INPUT_10_Pin ENC_INPUT_5_Pin ENC_INPUT_0_Pin */
  GPIO_InitStruct.Pin = ENC_INPUT_10_Pin|ENC_INPUT_5_Pin|ENC_INPUT_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC3 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_INPUT_7_Pin ENC_INPUT_9_Pin ENC_INPUT_3_Pin */
  GPIO_InitStruct.Pin = ENC_INPUT_7_Pin|ENC_INPUT_9_Pin|ENC_INPUT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DR_RW_Pin */
  GPIO_InitStruct.Pin = DR_RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DR_RW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI3_CS_Pin */
  GPIO_InitStruct.Pin = SPI3_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI3_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc1)
{
//	drts = DMA2_Stream0->NDTR;	// ЭТА ШТУКА ГОВОРИТ О КОЛИЧЕСТВЕ ГОТОВЫХ ДАННЫХ ПОД ОТПРАВКУ

	// Сюда добавить условия какая это плата: данные по uart или по usb
	if(!ThisDeviceOnUsartCtrl)
	{
		DMA2_Stream0->NDTR;
		CDC_Transmit_FS(&ADC_Data[0], drts);
		dstc += drts;
		glb_dstc += drts;
	}
	else if(ThisDeviceOnUsartCtrl)
	{
		DMA2_Stream0->NDTR;
		HAL_UART_Transmit_IT(&huart6, &ADC_Data[0], drts);
		dstc += drts;
		glb_dstc += drts;
	}
	else if(!(ThisDeviceOnUsartCtrl) && (ETEMode_Enable == true))
	{
		CDC_Transmit_FS(&ADC_Data[0], drts);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
//	drts = DMA2_Stream0->NDTR;
	if(!ThisDeviceOnUsartCtrl)
	{
		DMA2_Stream0->NDTR;
		CDC_Transmit_FS(&ADC_Data[drts], drts);
		dstc += drts;
		glb_dstc += drts;
	}
	else if(ThisDeviceOnUsartCtrl)
	{
		DMA2_Stream0->NDTR;
		HAL_UART_Transmit_IT(&huart6, &ADC_Data[drts], drts);
		dstc += drts;
		glb_dstc += drts;
	}
	else if(!(ThisDeviceOnUsartCtrl) && (ETEMode_Enable == true))
	{

	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
	{
		if (HAL_GPIO_ReadPin(Motor[0].md_encod_sn.GPIOsupSens, Motor[0].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
		{
			EncTimeNow[0] = Motor[0].md_drum_cnt;
			EncCntNow[0] = Motor[0].md_encod_sn.cnt;
			if(Motor[0].md_encod_sn.cnt == 0)
			{
//				ContRegulatorValue = 50;
				EncTimeOld[0] = EncTimeNow[0];
			}
			if(EncTimeOld[0] != EncTimeNow[0])
			{
//				EncTime[0] += (EncTimeNow[0] - EncTimeOld[0]);
				d_EncTime[0][Motor[0].md_encod_sn.cnt] = (EncTimeNow[0] - EncTimeOld[0]);

				RegVal[Motor[0].md_encod_sn.cnt] = Coef_P * ((((1.0 * Motor[0].md_FL2_Angle) / 1.5) / (Motor[0].md_FL2_Time * 0.01))
						- (10000.0 / (1.0 * (d_EncTime[0][Motor[0].md_encod_sn.cnt]))));
//						+ (RegValOld[Motor[0].md_encod_sn.cnt - 1] + (Coef_I * RegVal[Motor[0].md_encod_sn.cnt]));


				int16_t temp = ContRegulatorValue + (int16_t)(RegVal[Motor[0].md_encod_sn.cnt]);
				if(temp >= 900) ContRegulatorValue = 900;
				else if(temp <= 50) ContRegulatorValue = 50;
				else ContRegulatorValue = temp;


				SpeedAngleMas[Motor[0].md_CountDataToRecv] = ContRegulatorValue;
				Motor[0].md_CountDataToRecv++;
			}


			EncTimeOld[0] = EncTimeNow[0];
//			d_EncCnt[0] = EncCntNow[0] - EncCntOld[0];
//			EncCnt[0] += EncCntNow[0] - EncCntOld[0];
//			EncCntOld[0] = EncCntNow[0];
			Motor[0].md_encod_sn.cnt++;

			if(Motor[0].TOM == WRM_ANGLE_MODE)
			{
				if(Motor[0].md_st == WORKING)
				{
					if(ContRegulatorValue < 0)
					{
						Motor[0].md_chl_value = ContRegulatorValue;
						Motor[0].md_chr_value = 0;
					}
					else if(ContRegulatorValue > 0)
					{
						Motor[0].md_chr_value = ContRegulatorValue;
						Motor[0].md_chl_value = 0;
					}
					else if(ContRegulatorValue == 0)
					{
						Motor[0].md_chr_value = 1000;
						Motor[0].md_chl_value = 1000;
					}
					if(Motor[0].md_prch == PAIRCHANNEL_1)
					{
						Motor[0].md_htim->Instance->CCR1 = Motor[0].md_chl_value;
						Motor[0].md_htim->Instance->CCR2 = Motor[0].md_chr_value;
					}
					else if(Motor[0].md_prch == PAIRCHANNEL_2)
					{
						Motor[0].md_htim->Instance->CCR3 = Motor[0].md_chl_value;
						Motor[0].md_htim->Instance->CCR4 = Motor[0].md_chr_value;
					}
				}
			}
			flag_motor_is_move = true;
		}
		else if (HAL_GPIO_ReadPin(Motor[0].md_encod_sn.GPIOsupSens, Motor[0].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
		{
//			if (Motor[0].md_encod_sn.dir != RIGHT) Motor[0].md_encod_sn.cnt = 0;
			Motor[0].md_encod_sn.dir = RIGHT;

		}
//	if (GPIO_Pin == GPIO_PIN_14)
//	{
//		if (HAL_GPIO_ReadPin(Motor[1].md_encod_sn.GPIOsupSens, Motor[1].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
//		{
//			if(Motor[1].md_encod_sn.dir != LEFT)
//			{
//				Motor[1].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[1].md_encod_sn.cnt++;
//				Motor[1].md_encod_sn.dir = LEFT;
//			}
//		}
//		else if (HAL_GPIO_ReadPin(Motor[1].md_encod_sn.GPIOsupSens, Motor[1].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
//		{
//			if(Motor[1].md_encod_sn.dir != RIGHT)
//			{
//				Motor[1].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[1].md_encod_sn.cnt++;
//				Motor[1].md_encod_sn.dir = RIGHT;
//			}
//		}
//	}
//
//	if (GPIO_Pin == GPIO_PIN_12)
//	{
//		if (HAL_GPIO_ReadPin(Motor[2].md_encod_sn.GPIOsupSens, Motor[2].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
//		{
//			if(Motor[2].md_encod_sn.dir != LEFT)
//			{
//				Motor[2].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[2].md_encod_sn.cnt++;
//				Motor[2].md_encod_sn.dir = LEFT;
//			}
//		}
//		else if (HAL_GPIO_ReadPin(Motor[2].md_encod_sn.GPIOsupSens, Motor[2].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
//		{
//			if(Motor[2].md_encod_sn.dir != RIGHT)
//			{
//				Motor[2].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[2].md_encod_sn.cnt++;
//				Motor[2].md_encod_sn.dir = RIGHT;
//			}
//		}
//	}
//
//	if (GPIO_Pin == GPIO_PIN_5)
//	{
//		if (HAL_GPIO_ReadPin(Motor[3].md_encod_sn.GPIOsupSens, Motor[3].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
//		{
//			if(Motor[3].md_encod_sn.dir != LEFT)
//			{
//				Motor[3].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[3].md_encod_sn.cnt++;
//				Motor[3].md_encod_sn.dir = LEFT;
//			}
//		}
//		else if (HAL_GPIO_ReadPin(Motor[3].md_encod_sn.GPIOsupSens, Motor[3].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
//		{
//			if(Motor[3].md_encod_sn.dir != RIGHT)
//			{
//				Motor[3].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[3].md_encod_sn.cnt++;
//				Motor[3].md_encod_sn.dir = RIGHT;
//			}
//		}
//	}
//
//	if (GPIO_Pin == GPIO_PIN_3)
//	{
//		if (HAL_GPIO_ReadPin(Motor[4].md_encod_sn.GPIOsupSens, Motor[4].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
//		{
//			if(Motor[4].md_encod_sn.dir != LEFT)
//			{
//				Motor[4].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[4].md_encod_sn.cnt++;
//				Motor[4].md_encod_sn.dir = LEFT;
//			}
//		}
//		else if (HAL_GPIO_ReadPin(Motor[4].md_encod_sn.GPIOsupSens, Motor[4].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
//		{
//			if(Motor[4].md_encod_sn.dir != RIGHT)
//			{
//				Motor[4].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[4].md_encod_sn.cnt++;
//				Motor[4].md_encod_sn.dir = RIGHT;
//			}
//		}
//	}
//
//	if (GPIO_Pin == GPIO_PIN_1)
//	{
//		if (HAL_GPIO_ReadPin(Motor[5].md_encod_sn.GPIOsupSens, Motor[5].md_encod_sn.PINsupSens) == GPIO_PIN_SET)
//		{
//			if(Motor[5].md_encod_sn.dir != LEFT)
//			{
//				Motor[5].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[5].md_encod_sn.cnt++;
//				Motor[5].md_encod_sn.dir = LEFT;
//			}
//		}
//		else if (HAL_GPIO_ReadPin(Motor[5].md_encod_sn.GPIOsupSens, Motor[5].md_encod_sn.PINsupSens) == GPIO_PIN_RESET)
//		{
//			if(Motor[5].md_encod_sn.dir != RIGHT)
//			{
//				Motor[5].md_encod_sn.cnt = 1;
//			}
//			else
//			{
//				Motor[5].md_encod_sn.cnt++;
//				Motor[5].md_encod_sn.dir = RIGHT;
//			}
//		}
	}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)
	{
		// Далее переписать в доп протокол между STM
		// Тип отправить что то, по нему определять что полетит далее данные или команда или еще какая нибудь хрень
		UsartData[UsartDataCnt2] = UsartDataByte;
		UsartDataCnt2++;
		if(UsartData[0] == UsartDataCnt2)
		{

			char data[50];
			for(uint8_t i = 0; i < UsartDataCnt2; i++) data[i] = UsartData[i + 1];

			HandProtezRecvInstructionCorrectToReverse((uint8_t*)&data, UsartDataCnt2 - 1);
			HandProtezRecvInstruction((uint8_t*)data, UsartDataCnt2 - 1);

//			memset(UsartData, '\0', UsartDataCnt2);
			UsartDataCnt2 = 0;
		}
		HAL_UART_Receive_IT(&huart6, &UsartDataByte, 1);
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
