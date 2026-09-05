/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct usb_command_t {
	uint8_t 	number_motor;
	float 		angle_trg;
	uint8_t		op_cmd;
	uint32_t	speed;
} usb_command_t;
typedef struct lim_t {
	float lim_min;
	float lim_max;
} lim_t ;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOB
#define SCK_EN_Pin GPIO_PIN_1
#define SCK_EN_GPIO_Port GPIOB
#define M1_I2_Pin GPIO_PIN_15
#define M1_I2_GPIO_Port GPIOA
#define M1_I1_Pin GPIO_PIN_3
#define M1_I1_GPIO_Port GPIOB
#define M2_I2_Pin GPIO_PIN_4
#define M2_I2_GPIO_Port GPIOB
#define M2_I1_Pin GPIO_PIN_5
#define M2_I1_GPIO_Port GPIOB
#define M3_I2_Pin GPIO_PIN_6
#define M3_I2_GPIO_Port GPIOB
#define M3_I1_Pin GPIO_PIN_7
#define M3_I1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
