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
#include "../ProtezLib/ProtezHandControl.h"
#include "../ProtezLib/ProtezHandUsbProtocol.h"
#include "../ProtezLib/ProtezHandADC.h"
#include "../ProtezLib/ProtezHandEncoder.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "stm32f4xx_hal_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*Макросы энкодеров*/
#define ENC_GPIO_1	GPIOB
#define ENC_PIN_1	GPIO_PIN_10
#define ENC_GPIO_2	GPIOB
#define ENC_PIN_2	GPIO_PIN_14
#define ENC_GPIO_3	GPIOB
#define ENC_PIN_3	GPIO_PIN_13
#define ENC_GPIO_4	GPIOC
#define ENC_PIN_4	GPIO_PIN_5
#define ENC_GPIO_5	GPIOC
#define ENC_PIN_5	GPIO_PIN_3
#define ENC_GPIO_6	GPIOC
#define ENC_PIN_6	GPIO_PIN_1

#define SUP_ENC_GPIO_1	GPIOB
#define SUP_ENC_PIN_1	GPIO_PIN_2
#define SUP_ENC_GPIO_2	GPIOB
#define SUP_ENC_PIN_2	GPIO_PIN_15
#define SUP_ENC_GPIO_3	GPIOB
#define SUP_ENC_PIN_3	GPIO_PIN_12
#define SUP_ENC_GPIO_4	GPIOC
#define SUP_ENC_PIN_4	GPIO_PIN_4
#define SUP_ENC_GPIO_5	GPIOC
#define SUP_ENC_PIN_5	GPIO_PIN_2
#define SUP_ENC_GPIO_6	GPIOC
#define SUP_ENC_PIN_6	GPIO_PIN_0
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
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

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

PrHand_Motor_typedef Motor[6] =
{
	PrHand_Motor_typedef(0),
	PrHand_Motor_typedef(1),
	PrHand_Motor_typedef(2),
	PrHand_Motor_typedef(3),
	PrHand_Motor_typedef(4),
	PrHand_Motor_typedef(5),
};
ProtezHandADC PrHand_ADC(&hadc1, &htim2);
uint32_t GLB_Time[3];


uint32_t UsartDataCount;
uint8_t UsartDataByte;
uint8_t UsartRecvData[5000];
_Bool flag_uartResending;

typedef struct _tim_irq
{
	uint32_t counter_1;
	uint32_t counter_2;
	uint32_t counter_3;

} _tim_irq;
_tim_irq tim_irq;
typedef struct _cnt_instrument
{
	uint32_t cnt_instr_1;
	uint32_t cnt_instr_2;
	uint32_t cnt_instr_3;

} _cnt_instrument;
_cnt_instrument cnt_instrument;




void ProtezInit()
{
	ProtezHandUsbProtocol::setUARTHandle(&huart6);
	ProtezHandUsbProtocol::setTimerTransmiter(&htim9);
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)&UsartDataByte, 1);

	PrHand_ADC.setPoints(500);
	PrHand_ADC.setPackSizeData(20);

	Motor[0].setTIM(&htim3);
	Motor[0].setChannel({PR_CHANNEL_3, PR_CHANNEL_4}, PR_DIR_FORWARD);
	Motor[0].cls_encoder.setAllGPIO(ENC_GPIO_1, ENC_PIN_1, SUP_ENC_GPIO_1, SUP_ENC_PIN_1, GPIO_PIN_RESET);
	Motor[0].setADCChannel(ADC_Channel_5);
	Motor[0].setTargetSide(PR_MS_Stop);
	Motor[0].setPWM(0);

	Motor[1].setTIM(&htim4);
	Motor[1].setChannel({PR_CHANNEL_1, PR_CHANNEL_2}, PR_DIR_FORWARD);
	Motor[1].cls_encoder.setAllGPIO(ENC_GPIO_2, ENC_PIN_2, SUP_ENC_GPIO_2, SUP_ENC_PIN_2, GPIO_PIN_SET);
	Motor[1].setADCChannel(ADC_Channel_3);
	Motor[1].setTargetSide(PR_MS_Stop);
	Motor[1].setPWM(0);

	Motor[2].setTIM(&htim4);
	Motor[2].setChannel({PR_CHANNEL_3, PR_CHANNEL_4}, PR_DIR_FORWARD);
	Motor[2].cls_encoder.setAllGPIO(ENC_GPIO_3, ENC_PIN_3, SUP_ENC_GPIO_3, SUP_ENC_PIN_3, GPIO_PIN_SET);
	Motor[2].setADCChannel(ADC_Channel_4);
	Motor[2].setTargetSide(PR_MS_Stop);
	Motor[2].setPWM(0);

	Motor[3].setTIM(&htim1);
	Motor[3].setChannel({PR_CHANNEL_1, PR_CHANNEL_2}, PR_DIR_FORWARD);
	Motor[3].cls_encoder.setAllGPIO(ENC_GPIO_4, ENC_PIN_4, SUP_ENC_GPIO_4, SUP_ENC_PIN_4, GPIO_PIN_SET);
	Motor[3].setADCChannel(ADC_Channel_2);
	Motor[3].setTargetSide(PR_MS_Stop);
	Motor[3].setPWM(0);

	Motor[4].setTIM(&htim3);
	Motor[4].setChannel({PR_CHANNEL_1, PR_CHANNEL_2}, PR_DIR_FORWARD);
	Motor[4].cls_encoder.setAllGPIO(ENC_GPIO_5, ENC_PIN_5, SUP_ENC_GPIO_5, SUP_ENC_PIN_5, GPIO_PIN_SET);
	Motor[4].setADCChannel(ADC_Channel_7);
	Motor[4].setTargetSide(PR_MS_Stop);
	Motor[4].setPWM(0);

	Motor[5].setTIM(&htim5);
	Motor[5].setChannel({PR_CHANNEL_1, PR_CHANNEL_2}, PR_DIR_FORWARD);
	Motor[5].cls_encoder.setAllGPIO(ENC_GPIO_6, ENC_PIN_6, SUP_ENC_GPIO_6, SUP_ENC_PIN_6, GPIO_PIN_SET);
	Motor[5].setADCChannel(ADC_Channel_6);
	Motor[5].setTargetSide(PR_MS_Stop);
	Motor[5].setPWM(0);
}






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
//  MX_ADC1_Init();
//  MX_TIM2_Init();
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

  ProtezInit();
  PrHand_Motor_typedef::SetTIMHandlerInstr(&htim11);

  DRIVER_CTRL_ON;
//  DRIVER_CTRL_OFF;
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim10);
//  PrHand_Motor_typedef::StartTIM(&htim3);

  PrHand_Motor_typedef::StartTIMHandlerInstr();




  /* USER CODE END 2 */

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
  htim9.Init.Prescaler = 60-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 10-1;
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
  huart6.Init.BaudRate = 200000;
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
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	ProtezHandUsbProtocol::FlagDataADC = true;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{



}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{

	}
	else if (htim->Instance == TIM2)
	{
		uint8_t t = 0;
		uint32_t x = 0;
		for (auto &mot : Motor)
		{
			if (mot.getEnabledADC() == _ADC_Motor_Enable)
			{
				x = mot.cls_encoder.getCounter();
				ProtezHandEncoder::ImpulsData[t++] = (uint8_t)(x & 0x0000FF);
				ProtezHandEncoder::ImpulsData[t++] = (uint8_t)((x & 0x00FF00) >> 8);
			}
		}



	}
	else if (htim->Instance == TIM3)
	{

	}
	else if (htim->Instance == TIM4)
	{

	}
	else if (htim->Instance == TIM5)
	{

	}
	else if (htim->Instance == TIM9) // ТАЙМЕР ОТВЕЧАЮЩИЙ ЗА ОТПРАВКУ
	{
		ProtezHandUsbProtocol::tim_tx_counter_tick++;
		/*Отправляем данные АЦП*/
		/*********************************************/
		/*********************************************/
		if (ProtezHandUsbProtocol::FlagDataADC)
		{
			if (ProtezHandUsbProtocol::FlagUartControl)
			{
				HAL_UART_Transmit_IT(&huart6, (uint8_t*)&ProtezHandADC::DataADC, PrHand_ADC.getPackSizeData() * 2);
			}
			else
			{
				for (auto &mot : Motor)
				{
					if (mot.getEnabledADC() == _ADC_Motor_Enable)
					{
						/*Собираем данные с ацп и энкодера*/
						ProtezHandUsbProtocol::datactrl.out_sub_data[*ProtezHandUsbProtocol::datactrl.ptr_adc_data] = ProtezHandADC::DataADC[0];
						ProtezHandUsbProtocol::datactrl.out_sub_data[*ProtezHandUsbProtocol::datactrl.ptr_speed_data] = ProtezHandEncoder::ImpulsData[0];
					}
				}

				CDC_Transmit_FS((uint8_t*)&ProtezHandUsbProtocol::datactrl.out_sub_data, ProtezHandUsbProtocol::datactrl.count_sub_data);
				ProtezHandUsbProtocol::datactrl.count_out_data = 0;
			}
			ProtezHandUsbProtocol::FlagDataADC = false;
		}
		/*********************************************/
		/*Отправляем данные скорости вращения вала двигателя*/
		else if (ProtezHandUsbProtocol::FlagDataSPEED)
		{
			if (ProtezHandUsbProtocol::FlagUartControl)
			{
//				HAL_UART_Transmit_IT(&huart6, (uint8_t*)&ProtezHandADC::DataADC, PrHand_ADC.getPackSizeData() * 2);
			}
			else
			{
//				CDC_Transmit_FS((uint8_t*)&ProtezHandADC::DataADC, PrHand_ADC.getPackSizeData() * 2);
			}
		}
		/*********************************************/
		/*********************************************/
		/***************COLLECTING DATA***************/
//		if (tim_irq.counter_1 == (PrHand_Motor_typedef::MaxTimeInterval / ProtezHandEncoder::getCntPoint()))
//		{
//			tim_irq.counter_1 = 0;
//			for (auto &mot : Motor)
//			{
//				if (mot.getState() == _Working)
//				{
//					auto x = mot.cls_encoder.getCounter();
//					auto y = mot.cls_encoder.getTimePoint();
//					mot.cls_encoder.DataSPEED[mot.cls_encoder.indexDataSPEED] = (x - cnt_instrument.cnt_instr_1) / (y - cnt_instrument.cnt_instr_2 + 0.00001) * 10000.0;
//					mot.cls_encoder.indexDataSPEED++;
//
//
//					cnt_instrument.cnt_instr_1 = x;
//					cnt_instrument.cnt_instr_2 = y;
//				}
//			}
//		}
		/*********************************************/
		/*********************************************/
		if (ProtezHandUsbProtocol::tim_tx_counter_tick >= 100)	// 1ms
		{
			ProtezHandUsbProtocol::tim_tx_counter_tick = 0;
			ProtezHandUsbProtocol::tim_tx_counter_ms++;
			tim_irq.counter_1++;
		}
		if (ProtezHandUsbProtocol::tim_tx_counter_ms >= 1000)	// 1s
		{
			ProtezHandUsbProtocol::tim_tx_counter_ms = 0;
			ProtezHandUsbProtocol::tim_tx_counter_s++;
		}
		/*********************************************/
		/*********************************************/
	}
	else if (htim->Instance == TIM10)
	{
		if ((GLB_Time[2] != 0) && (GLB_Time[2] % 1000) == 0)
		{
			GLB_Time[0]++;		// 1 s
		}
		if ((GLB_Time[2] != 0) && (GLB_Time[2] % 10) == 0)
		{
			GLB_Time[1]++;		// 1 ms
		}
		GLB_Time[2]++;			// 100 us



//		if (GLB_Time[2] == 65535) GLB_Time[2] = 0;	// 0.01 second
//		if (GLB_Time[1] == 65535) GLB_Time[1] = 0;	// 0.1 second
//		if (GLB_Time[0] == 65535) GLB_Time[0] = 0;	// 1.0 second
	}
	else if (htim->Instance == TIM11) // ТАЙМЕР ОБРАБАТЫВАЮЩИЙ НАСТРОЙКУ И ВКЛЮЧЕНИЕ ДВИГАТЕЛЕЙ
	{
		for (auto &motor : Motor) {
			if (motor.CheckWorkInterval() == _Launched)
			{

			}
			else if (motor.CheckWorkInterval() == _Working)
			{
				if ((PrHand_ADC.getStateADC() == _ADC_Disable) || (PrHand_ADC.getStateADC() == _ADC_Released)) {
					if (PrHand_ADC.getEnabledADC() == _ADC_NoConfigured) {
						PrHand_ADC.ADC_Init();
					}
			}

				if (motor.getState() == _Launched) {
					/*Если хотябы одна система передачи включена*/
					if ((PrHand_ADC.getStateADC() == _ADC_Configured) /*|| (Feedback Enabled)*/)
					{
						ProtezHandUsbProtocol::TimerTX_Start();
						motor.cls_encoder.indexDataSPEED = 0;
					}

					if (PrHand_ADC.getStateADC() == _ADC_Configured)
					{
						if (ProtezHandUsbProtocol::FlagUartControl)
						{
							ProtezHandUsbProtocol::UsartProtocol::transmitUartDataStartPack();
						}
						else ProtezHandUsbProtocol::transmitStartPackData();
						PrHand_ADC.StartADC();
						ProtezHandEncoder::setEnable(true);
					}

					motor.Start();
					motor.setState(_Working);
				}

			}
			else if (motor.CheckWorkInterval() == _Ending) {
				if (motor.getState() == _Working) {
					motor.Stop();

					if (PrHand_ADC.getStateADC() == _ADC_Working)
					{
						PrHand_ADC.StopADC();
						ProtezHandEncoder::setEnable(false);
						if (ProtezHandUsbProtocol::FlagUartControl)
						{
							ProtezHandUsbProtocol::UsartProtocol::transmitUartDataStopPack();
						}
						else ProtezHandUsbProtocol::transmitStopPackData();
						if(ProtezHandUsbProtocol::FlagUartControl) ProtezHandUsbProtocol::FlagUartControl = false;
					}

					/*Если все системы передачи выключины*/
					if ((PrHand_ADC.getStateADC() == _ADC_Released) /*Feedback disabled*/)
					{
						ProtezHandUsbProtocol::TimerTX_Stop();
					}
				}
			}
		}
		PrHand_Motor_typedef::setTIMHandlerInstrCounter((PrHand_Motor_typedef::getTIMHandlerInstrCounter() + 1));
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6) {
		UsartRecvData[UsartDataCount] = UsartDataByte;

		/*************************************************************************************************/
		/*RecvCommand*/
		if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_USART_START.first)
				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_USART_START.second)) {
			UsartDataCount++;
		}
		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_USART_STOP.first)
				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_USART_STOP.second)) {
			std::vector<uint8_t> temp(0);
			uint16_t localusartcount = 2;
			uint16_t count_dat = 0;
			uint8_t code = 0;
			uint16_t cc = 2;
			while ((UsartDataCount - 2) > localusartcount) {
				code = UsartRecvData[cc];
				count_dat = (code & 0xE0) >> 5;
				code = (code & 0x1F);
				cc++;
				localusartcount++;
				for (uint16_t r = cc; r < cc + count_dat; r++) {
					temp.push_back(UsartRecvData[r]);
					localusartcount++;
				}
				cc += count_dat;
				ProtezHandUsbProtocol::setCommand[code] = temp;
				temp.clear();
			}
			ProtezHandUsbProtocol::FlagUartControl = true;
			ProtezHandUsbProtocol::UsartProtocol::UsartCommand();
			UsartDataCount = 0;
		}
		/*************************************************************************************************/
		/*ADC_DATA*/
		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_START_pair.first)
				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_START_pair.second)) {
			flag_uartResending = true;
			UsartDataCount++;
		}
		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_STOP_pair.first)
				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_STOP_pair.second)) {
			CDC_Transmit_FS((uint8_t*)&UsartRecvData, UsartDataCount + 1);
			flag_uartResending = false;
			UsartDataCount = 0;
		}
		else if (flag_uartResending)
		{
			if (UsartDataCount >= PrHand_ADC.getPackSizeData() * 2)
			{
				CDC_Transmit_FS((uint8_t*)&UsartRecvData, UsartDataCount + 1);
				UsartDataCount = 0;
			}
			else UsartDataCount++;
		}
		/*************************************************************************************************/
		/*SPEED_DATA*/
//		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_START_pair.first)
//				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_START_pair.second)) {
//			flag_uartResending = true;
//			UsartDataCount++;
//		}
//		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_STOP_pair.first)
//				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_STOP_pair.second)) {
//		}
		else UsartDataCount++;

	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (ProtezHandEncoder::getEnable())
	{
		PrHand_Motor_typedef *t_mot = 0;
		for (auto &mot : Motor) {
			if (mot.cls_encoder.getPINmain() == GPIO_Pin) {
				t_mot = &mot;
				break;
			}
		}

		if (HAL_GPIO_ReadPin(t_mot->cls_encoder.getPORsup(), t_mot->cls_encoder.getPINsup()) == t_mot->cls_encoder.PositiveSideRotate)
		{
			auto x = t_mot->cls_encoder.getCounter() + 1;
			t_mot->cls_encoder.setCounter(x);
			t_mot->cls_encoder.setTimePoint(GLB_Time[2]);

		}
		else
		{

		}

	}
	/*************************************************************************************************/


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
