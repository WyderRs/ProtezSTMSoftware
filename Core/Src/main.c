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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include "StarVar.h"

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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

typedef struct Point
{
    double x;
    double y;
} Point;

enum p_idx{
    p0 = 0,     // K
    p1, p2,     // O, P1
    p3, p4,     // C1, B1
    p5, p6,     // A1, N
    p7, p8,     // A, M
    p9, p10,    // B, L
    p11,        // C
};

typedef struct exoskeleton_t {
	Point p[13];

	float current_q[3];
} exoskeleton_t;









typedef enum direction_t {
	forward = 0,
	backforward,
} direction_t;
typedef struct PID_t {
	float transfer_value;	/* 1 gradus = 1 pulse*/

	float koef_p;


	float old_current_angle;
	float current_angle;
	uint32_t old_current_time;
	uint32_t current_time;

	float trg_angle;
	float trg_time;

	float current_velocity;
	float trg_velocity;


} PID_t;

typedef struct motor_t {
	GPIO_TypeDef*	gpio1;
	uint32_t 		pin1;
	GPIO_TypeDef*	gpio2;
	uint32_t 		pin2;

	uint32_t 		counter;
	uint32_t 		trg_up;
	uint32_t		pwm_count_down;

	_Bool			motor_en;
	direction_t		direction;

	_Bool			enc_ch_state1;
	_Bool			enc_ch_state2;

	uint32_t 		counter_enc;

	PID_t			pid;

	float 			linear_step_on_per;
} motor_t;






motor_t motor[3];
uint8_t data;
uint8_t USB_Angle[3];

exoskeleton_t exoskeleton;


/* Длина линии по точкам */
double lenPP(Point a, Point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
/* Угол по трем сторонам */
double angle3Line(double l1, double l2, double l3) {
    return acos((((l2 * l2) + (l3 * l3) - (l1 * l1)) / (2.0 * l2 * l3) ));
}
/* Модуль по углу и двум сторонам */
double moduleALL(double angle, double line1, double line2) {
    return sqrt((line1 * line1) + (line2 * line2) - 2.0 * line1 * line2 * cos(angle));
}



void received_command() {

	float sup_angle2[30] = {0, };
	float fi[3] = {0, };

    /*============================================================*/


	/* Имеем разворот (классическое положение) */
	fi[0] = 3.1415926536;
	fi[1] = 3.1415926536;
	fi[2] = 3.1415926536;

    fi[0] += (USB_Angle[0] * 3.14) / 180.0;
    fi[1] += (USB_Angle[1] * 3.14) / 180.0;
    fi[2] += (USB_Angle[2] * 3.14) / 180.0;

    exoskeleton.p[p0].x		= p0x;
    exoskeleton.p[p0].y		= p0y;
    exoskeleton.p[p1].x		= exoskeleton.p[p0].x + lx_p0p1;
    exoskeleton.p[p1].y		= exoskeleton.p[p0].y + ly_p0p1;
    exoskeleton.p[p2].x		= exoskeleton.p[p0].x + lx_p0p1 + lx_p1p2;
    exoskeleton.p[p2].y		= exoskeleton.p[p0].y + ly_p0p1 + ly_p1p2;


    l_p0p1					= lenPP(exoskeleton.p[p0], exoskeleton.p[p1]);
    l_p0p11					= l_p0p10 / 2.0;
    l_p9p10					= l_p8p10 / 2.0;
    l_p7p8					= l_p6p8 / 2.0;
    l_p10p11        		= l_p0p10 / 2.0;
    l_p8p9					= l_p8p10 / 2.0;




    /* p11 */       /*OK*/
    exoskeleton.p[p11].x        = l_p0p11 * cos(fi[0]);
    exoskeleton.p[p11].y        = l_p0p11 * sin(fi[0]);
    /* p10 */       /*OK*/
    exoskeleton.p[p10].x        = l_p0p10 * cos(fi[0]);
    exoskeleton.p[p10].y        = l_p0p10 * sin(fi[0]);
    /* p9 */        /*OK*/
    exoskeleton.p[p9].x        = exoskeleton.p[p10].x + l_p9p10 * cos(fi[0] + (fi[1] - PI));
    exoskeleton.p[p9].y        = exoskeleton.p[p10].y + l_p9p10 * sin(fi[0] + (fi[1] - PI));
    /* p8 */        /*OK*/
    exoskeleton.p[p8].x        = exoskeleton.p[p10].x + l_p8p10 * cos(fi[0] + (fi[1] - PI));
    exoskeleton.p[p8].y        = exoskeleton.p[p10].y + l_p8p10 * sin(fi[0] + (fi[1] - PI));
    /* p7 */        /*OK*/
    exoskeleton.p[p7].x        = exoskeleton.p[p8].x + l_p7p8 * cos(fi[0] + (fi[1] - PI) + (fi[2] - PI));
    exoskeleton.p[p7].y        = exoskeleton.p[p8].y + l_p7p8 * sin(fi[0] + (fi[1] - PI) + (fi[2] - PI));
    /* p6 */        /*OK*/
    exoskeleton.p[p6].x        = exoskeleton.p[p8].x + l_p6p8 * cos(fi[0] + (fi[1] - PI) + (fi[2] - PI));
    exoskeleton.p[p6].y        = exoskeleton.p[p8].y + l_p6p8 * sin(fi[0] + (fi[1] - PI) + (fi[2] - PI));
    /* p3 */
    double l_p1p11 = lenPP(exoskeleton.p[p1], exoskeleton.p[p11]);
    sup_angle2[18] = angle3Line(l_p1p11, l_p0p11, l_p0p1);
    sup_angle2[19] = angle3Line(l_p1p3, l_p1p11, l_p3p11) + angle3Line(l_p0p1, l_p1p11, l_p0p11);    /* ЗАВИСИМОСТЬ ОТ ЗНАКА +- */
                                                                                                    /* + ЕСЛИ ОТ НУЛЯ fi[0] */
    exoskeleton.p[p3].x         = exoskeleton.p[p11].x + l_p3p11 * cos((fi[0] - PI) + sup_angle2[19]);
    exoskeleton.p[p3].y         = exoskeleton.p[p11].y + l_p3p11 * sin((fi[0] - PI) + sup_angle2[19]);

    l_p2p3       = lenPP(exoskeleton.p[p3], exoskeleton.p[p2]);       /* Q0 */
//    l_p3p4       = lenPP(exoskeleton.p[p3], exoskeleton.p[p4]);       /* Q1 */

    /******************/

    l_p0p3       = lenPP(exoskeleton.p[p0], exoskeleton.p[p3]);
    sup_angle2[1] = angle3Line(l_p0p3, l_p0p11, l_p3p11);

    double l_p9p11 = lenPP(exoskeleton.p[p9], exoskeleton.p[p11]);

    sup_angle2[20] = angle3Line(l_p4p11, l_p4p9, l_p9p11) - angle3Line(l_p10p11, l_p9p11, l_p9p10); /* ЗАВИСИМОСТЬ ОТ ЗНАКА +- */
                                                                                                    /* + ЕСЛИ ОТ НУЛЯ fi[0] */

    exoskeleton.p[p4].x         = exoskeleton.p[p9].x + l_p4p9 * cos((fi[0] - PI) + (fi[1] - PI) + sup_angle2[20]);
    exoskeleton.p[p4].y         = exoskeleton.p[p9].y + l_p4p9 * sin((fi[0] - PI) + (fi[1] - PI) + sup_angle2[20]);

    l_p3p4       = lenPP(exoskeleton.p[p3], exoskeleton.p[p4]);          /* Q1 */

    double l_p7p9 = lenPP(exoskeleton.p[p7], exoskeleton.p[p9]);
    sup_angle2[21] = angle3Line(l_p5p9, l_p7p9, l_p5p7) - angle3Line(l_p8p9, l_p7p8, l_p7p9);   /* ЗАВИСИМОСТЬ ОТ ЗНАКА +- */
                                                                                                /* + ЕСЛИ ОТ НУЛЯ fi[0] */

    exoskeleton.p[p5].x         = exoskeleton.p[p7].x + l_p5p7 * cos((fi[0] - PI) + (fi[1] - PI) + (fi[2] - PI) + sup_angle2[21]);
    exoskeleton.p[p5].y         = exoskeleton.p[p7].y + l_p5p7 * sin((fi[0] - PI) + (fi[1] - PI) + (fi[2] - PI) + sup_angle2[21]);

    l_p4p5          = lenPP(exoskeleton.p[p4], exoskeleton.p[p5]);       /* Q2 */


//    exoskeleton.current_q[0] = l_p2p3;
//    exoskeleton.current_q[1] = l_p3p4;
//    exoskeleton.current_q[2] = l_p4p5;

    /* Linear pitch value per screw rotation value */
    motor[0].linear_step_on_per = 0.35;

    /* Start */


    if (exoskeleton.current_q[0] < l_p2p3) {
    	motor[0].direction			= forward;
    }
    else motor[0].direction			= backforward;

    HAL_TIM_Base_Start_IT(&htim4);
    motor[0].motor_en = true;
    while(1) {

    	if (exoskeleton.current_q[0] == l_p2p3) {
    		HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);

    		motor[0].motor_en = false;
    		HAL_TIM_Base_Stop_IT(&htim4);
    	}
//    	if (exoskeleton.current_q[1] == l_p3p4) {
//
//    	}
//    	if (exoskeleton.current_q[2] == l_p4p5) {
//
//    	}

    }




//	if ((motor[0].pid.trg_angle - motor[0].pid.current_angle) <= 0) {
//		  motor[0].motor_en				= false;
//		  HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
//	}

}









/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */



  motor[0].gpio1 				= GPIOB;
  motor[0].pin1					= GPIO_PIN_7;
  motor[0].gpio2 				= GPIOB;
  motor[0].pin2					= GPIO_PIN_6;
  motor[0].pid.transfer_value	= 7.0;

  motor[0].direction			= forward;
  motor[0].motor_en				= false;
  motor[0].trg_up				= 1000;
  motor[0].pwm_count_down 		= 500;



  /* First position */
  exoskeleton.current_q[0] = 74.790899999999993;
  exoskeleton.current_q[1] = 59.259999999999998;
  exoskeleton.current_q[2] = 30.91;




  HAL_TIM_Base_Start_IT(&htim4);
//  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);


	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  for (uint32_t i = 0; i < 100; i++) {}
	  HAL_SPI_Receive_IT(&hspi1, &data, 1);
	  for (uint32_t i = 0; i < 100; i++) {}
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


	  if (!(data & (1 << 7))) motor[0].enc_ch_state2		= true;
	  else motor[0].enc_ch_state2							= false;
//	  current_q







	  //			motor[0].pid.current_angle 		= 2.4 * motor[0].counter_enc / motor[0].pid.transfer_value;
	  //			temp4								= (100000 * (motor[0].pid.current_angle - motor[0].pid.old_current_angle) /
	  //				  (motor[0].pid.current_time - motor[0].pid.old_current_time));
	  //			motor[0].pid.current_velocity 	= 100000 * (motor[0].pid.current_angle) / (motor[0].pid.current_time + 0.0001);
	  //			temp1 							= 0.08 * motor[0].pid.koef_p * (motor[0].pid.trg_velocity - motor[0].pid.current_velocity);
	  //			motor[0].pwm_count_down			+= temp1;
	  //			motor[0].pid.old_current_angle	= motor[0].pid.current_angle;
	  //			motor[0].pid.old_current_time		= motor[0].pid.current_time;
	  //			if (motor[0].pwm_count_down < 60) motor[0].pwm_count_down = 60;



/*
	  if (motor[0].counter_enc >= 10500)  //1050
	  {
		  motor[0].motor_en				= false;

		  HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
	  }
*/



	  HAL_Delay(20);


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10-1;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin|SCK_EN_Pin|M1_I1_Pin|M2_I2_Pin
                          |M2_I1_Pin|M3_I2_Pin|M3_I1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M1_I2_GPIO_Port, M1_I2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin SCK_EN_Pin M1_I1_Pin M2_I2_Pin
                           M2_I1_Pin M3_I2_Pin M3_I1_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|SCK_EN_Pin|M1_I1_Pin|M2_I2_Pin
                          |M2_I1_Pin|M3_I2_Pin|M3_I1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_I2_Pin */
  GPIO_InitStruct.Pin = M1_I2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_I2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
//
//
//
//
//
//
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim4)	// 10us
	{
		for (uint8_t i = 0; i < 3; i++) {
			if (motor[i].motor_en) {
				if (motor[i].counter >= (motor[i].trg_up - 1)) {
					if (motor[i].direction == forward) HAL_GPIO_TogglePin(motor[i].gpio1, motor[i].pin1);
					else if (motor[i].direction == backforward) HAL_GPIO_TogglePin(motor[i].gpio2, motor[i].pin2);
					motor[i].counter = 0;
				}
				else motor[i].counter++;

				if (motor[i].counter >= motor[i].pwm_count_down) {
					if (motor[i].direction == forward) HAL_GPIO_WritePin(motor[i].gpio1, motor[i].pin1, GPIO_PIN_RESET);
					else if (motor[i].direction == backforward) HAL_GPIO_WritePin(motor[i].gpio2, motor[i].pin2, GPIO_PIN_RESET);
				}
				motor[i].pid.current_time++;
			}
		}


//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//		  for (uint8_t i = 0; i < 8; i++) {
//			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//			  for(uint32_t w = 0; w < 1000; w++) {}
//			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//			  for(uint32_t w = 0; w < 1000; w++) {}
//
//			  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)) {
//				  data |= (1 << (7 - i));
//			  }
//			  else data &= ~(1 << (7 - i));
//		  }
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);


	}
	else if (htim == &htim3) {



	}

	for(uint32_t i = 0; i < 1000; i++) {}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_2) {
	}
	else if (GPIO_Pin == GPIO_PIN_10) {
	}
	else if (GPIO_Pin == GPIO_PIN_11) {
	}
	else if (GPIO_Pin == GPIO_PIN_9) {
		motor[0].enc_ch_state1		= true;

		if (motor[0].enc_ch_state1 && motor[0].enc_ch_state2) {
			motor[0].counter_enc++;

			motor[0].enc_ch_state1			= false;
			motor[0].enc_ch_state2			= false;

			exoskeleton.current_q[0]		+= motor[0].linear_step_on_per / 7.0;
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
