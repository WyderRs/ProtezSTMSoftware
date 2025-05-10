/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HandCotrol.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
volatile uint32_t GLB_Time[3];
extern MotorDefinition Motor[6];
extern PRGlbDef ProtezGlobalConf;
extern uint32_t glb_dstc;
extern uint32_t Target_dtsc[6];
extern uint32_t drts;
extern uint8_t ADC_Data[100];
extern uint16_t RegularValuePWM_PID[6];
extern bool FLAG_MotorIsMove[6];
uint16_t count_last_bytes;

extern uint8_t GLB_TypeCtrl;
extern FL2_TypeCtrlMove TCM;

extern uint32_t TEST_cntTim2;



extern uint16_t SpeedAngleMas[5000];

extern uint32_t d_EncTime[6][500];

extern uint32_t LimitCNT[6];
extern _Bool FLAG_MotorIsMove[6];
extern _Bool FlagDMA_START;


extern _Bool TransmitDataFlags[2];
extern _Bool ThisDeviceOnUsartCtrl;
extern uint32_t drts;
extern uint32_t dstc;
extern uint32_t drts_2;

extern uint8_t UsartData[120];

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

	if(!ThisDeviceOnUsartCtrl)
	{
		if(TransmitDataFlags[0] || TransmitDataFlags[1])
		{
			uint32_t temp = 0;
			if(TransmitDataFlags[1])
			{
				for(uint8_t i = 0; i < drts_2 * 2; i++)
				{
					ADC_Data[drts * 2 + i] = UsartData[i];
					temp++;
				}
				TransmitDataFlags[1] = false;
			}

			DMA2_Stream0->NDTR;
			CDC_Transmit_FS(&ADC_Data[0], drts * 2 + temp);
			dstc += drts * 2;
			glb_dstc += drts * 2;


			TransmitDataFlags[0] = false;


//			uint32_t Target_dtsc_temp = 0;
//			for(uint8_t ii = 0; ii < ProtezGlobalConf.NumMotorConfigured; ii++) Target_dtsc_temp += Target_dtsc[ii];
//			if(((Target_dtsc_temp - glb_dstc) > 0) && ((Target_dtsc_temp - glb_dstc) <Target_dtsc_temp))	// Device not undelivered data
//			{
//				if((glb_dstc % drts) > 0)
//				{
//					// ЗДЕСЬ НАДО БЫ ДЕЛИТЬ НА КОЛИЧЕСТВО ВКЛ АЦП (У МЕНЯ ПОКА 1)
//					count_last_bytes = DMA2_Stream0->NDTR - drts;
//					CDC_Transmit_FS(&ADC_Data[drts], count_last_bytes);
//				}
//				else
//				{
//					count_last_bytes = Target_dtsc_temp - glb_dstc;
//					CDC_Transmit_FS(&ADC_Data[0], count_last_bytes);
//					glb_dstc += count_last_bytes;
//				}
//			}
		}
	}




  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(Motor[i].TOM == WRM_ANGLE_MODE)
		{
			Motor[i].md_drum_cnt++;
			LimitCNT[i]++;
			if(Motor[i].md_st == WORKING)
			{
				if(FLAG_MotorIsMove[i] == false)
				{
					if(LimitCNT[i] > 300)
					{
						RegularValuePWM_PID[i] += 10;
						if(Motor[i].md_rotsd == Motor[i].Encoder.side[0]) FL_2_Motor_SetDuty(&Motor[i], RegularValuePWM_PID[i], 0);
						else if(Motor[i].md_rotsd == Motor[i].Encoder.side[1]) FL_2_Motor_SetDuty(&Motor[i], 0, RegularValuePWM_PID[i]);

						SpeedAngleMas[Motor[i].md_CountDataToRecv] = RegularValuePWM_PID[i];
						Motor[i].md_CountDataToRecv++;
						LimitCNT[i] = 0;
					}
				}
				FLAG_MotorIsMove[i] = false;
			}
		}
	}


  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */
	if ((GLB_Time[2] != 0) && (GLB_Time[2] % 100) == 0) GLB_Time[0]++;	// second
	if ((GLB_Time[2] != 0) && (GLB_Time[2] % 10) == 0) GLB_Time[1]++;	// 0.1 second
	GLB_Time[2]++;								// 0.01 second

	for (uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if (GLB_TypeCtrl == 0x01)	 /*PWM MODE*/
		{
			FL_1_Motor_ContinuousDuty(&Motor[i]);
			if ((Motor[i].md_st == WAITING) && (GLB_Time[2] >= Motor[i].md_startWorkTime))
			{
				if((ProtezGlobalConf.ADC_ChannelsEnable) && (!FlagDMA_START))
				{
					StartMeasurement();		// If there is enables channel ADC then to run measurement
					FlagDMA_START = true;
				}
				FL_1_Motor_Start(&Motor[i]);
			}
			else if ((Motor[i].md_st == WORKING) && (GLB_Time[2] >= Motor[i].md_stopWorkTime))
			{

				FL_1_Motor_Stop(&Motor[i]);
				if(CheckStateAllMotor() == FINISH)
				{
//					for(uint8_t ii = 0; ii < ProtezGlobalConf.NumMotorConfigured; ii++)
//					{
					uint32_t Target_dtsc_temp = 0;
					for(uint8_t ii = 0; ii < ProtezGlobalConf.NumMotorConfigured; ii++) Target_dtsc_temp += Target_dtsc[ii];
					if(((Target_dtsc_temp - glb_dstc) > 0) && ((Target_dtsc_temp - glb_dstc) <Target_dtsc_temp))	// Device not undelivered data
					{
						if((glb_dstc % drts) > 0)
						{
							// ЗДЕСЬ НАДО БЫ ДЕЛИТЬ НА КОЛИЧЕСТВО ВКЛ АЦП (У МЕНЯ ПОКА 1)
							count_last_bytes = DMA2_Stream0->NDTR - drts;
							CDC_Transmit_FS(&ADC_Data[drts], count_last_bytes);
						}
						else
						{
							count_last_bytes = Target_dtsc_temp - glb_dstc;
							CDC_Transmit_FS(&ADC_Data[0], count_last_bytes);
							glb_dstc += count_last_bytes;
						}
					}
//					}
					StopMeasurement();
					ProtezGlobalConf.ADC_ChannelsEnable = false;
					DRIVER_CTRL_OFF;
					GLB_TypeCtrl = 0x00;
				}
			}
		}
		else if (GLB_TypeCtrl == 0x02)	 /*ANGLE MODE*/
		{
			if (TCM == ANGLE_TIME)		// Angle-Time
			{
				if ((Motor[i].md_st == WAITING) && (GLB_Time[2] >= Motor[i].md_FL2_startWorkTime))
				{
					FLAG_MotorIsMove[i] = false;
					FL_2_Motor_Start(&Motor[i]);
				}
				else if ((Motor[i].md_st == WORKING) && /*(GLB_Time[2] < Motor[i].md_FL2_stopWorkTime) &&*/ (Motor[i].Encoder.CNT < (Motor[i].md_FL2_Angle / 1.5)))
				{
//					FL_2_Motor_ContinuousDuty(&Motor[i]);

				}
				else if (/*((Motor[i].md_st == WORKING) && (GLB_Time[2] >= Motor[i].md_FL2_stopWorkTime))
						|| */((Motor[i].Encoder.CNT > (Motor[i].md_FL2_Angle / 1.5)) && ((Motor[i].md_FL2_Angle / 1.5) != 0)))
				{
					FL_2_Motor_Stop(&Motor[i]);
					if(CheckStateAllMotor() == FINISH)
					{
//						for(uint8_t ii = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
//						{
//							if(((Target_dtsc[ii] - glb_dstc) > 0) && ((Target_dtsc[ii] - glb_dstc) < Target_dtsc[ii]))	// Device not undelivered data
//							{
//								if((glb_dstc % drts) > 0)
//								{
//									// ЗДЕСЬ НАДО БЫ ДЕЛИТЬ НА КОЛИЧЕСТВО ВКЛ АЦП (У МЕНЯ ПОКА 1)
//									count_last_bytes = DMA2_Stream0->NDTR - drts;
//									CDC_Transmit_FS(&ADC_Data[drts], count_last_bytes);
//								}
//								else
//								{
//									count_last_bytes = Target_dtsc[ii] - glb_dstc;
//									CDC_Transmit_FS(&ADC_Data[0], count_last_bytes);
//									glb_dstc += count_last_bytes;
//								}
//							}
//						}
//						StopMeasurement();
//						ProtezGlobalConf.ADC_ChannelsEnable = false;

//						DRIVER_CTRL_OFF;
						GLB_TypeCtrl = 0x00;
						RegularValuePWM_PID[i] = 0;
						if(Motor[i].md_EnableFeedBack == true)
						{
							CDC_Transmit_FS((uint8_t*)SpeedAngleMas, Motor[i].md_CountDataToRecv * 2);	 // 2 because data is uint16_t type
							Motor[i].md_CountDataToRecv = 0;
						}
						FLAG_MotorIsMove[i] = false;

					}
				}

			}
			else if (TCM == ANGLE_SPEED)	// Angle-Speed
			{
				FL_2_Motor_Start(&Motor[i]);
			}
			else if (TCM == TIME_SPEED)	// Time-Speed
			{
				FL_2_Motor_Start(&Motor[i]);
			}
		}
	}
	if (GLB_Time[2] == 65535) GLB_Time[2] = 0;
	if (GLB_Time[1] == 65535) GLB_Time[1] = 0;
	if (GLB_Time[0] == 65535) GLB_Time[0] = 0;

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (!(GLB_Time[0] >= 1))
	{
		TEST_cntTim2++;
	}


  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ENC_INPUT_7_Pin);
  HAL_GPIO_EXTI_IRQHandler(ENC_INPUT_9_Pin);
  HAL_GPIO_EXTI_IRQHandler(ENC_INPUT_3_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream6 global interrupt.
  */
void DMA2_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

  /* USER CODE END DMA2_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
  /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

  /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
