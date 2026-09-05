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
#include "StarVar.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
_Bool CMD_HOME;
_Bool CMD_MAIN;
_Bool CMD_GET;
_Bool CMD_HDSP;
_Bool CMD_LIM;
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





typedef enum cmd_t {
	MOVE = 0,
	HOLD,
	RELEASED,
} cmd_t;


typedef struct Point
{
    float x;
    float y;
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
//	float transfer_value;	/* 1 gradus = 1 pulse*/

//	float koef_p;


//	float old_current_angle;
//	float current_angle;
//	uint32_t old_current_time;
//	uint32_t current_time;

//	float trg_angle;
//	float trg_time;

//	float current_velocity;
//	float trg_velocity;


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

	_Bool 			enc_state;

	uint32_t 		counter_enc;

	PID_t			pid;

	float 			linear_step_on_per;
	float			transfer_value;

	float			lim_min;
	float			lim_max;
} motor_t;






motor_t motor[3];
uint8_t data;
volatile uint8_t USB_Angle[24];

exoskeleton_t 	exoskeleton;
usb_command_t 	u_cmd[3];
lim_t			u_lim[3];

float angle[3] = {0.0, };

uint8_t buffer_usb[100];


/* Длина линии по точкам */
float lenPP(Point a, Point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}
/* Угол по трем сторонам */
float angle3Line(float l1, float l2, float l3) {
    return acos((((l2 * l2) + (l3 * l3) - (l1 * l1)) / (2.0 * l2 * l3) ));
}
/* Модуль по углу и двум сторонам */
float moduleALL(float angle, float line1, float line2) {
    return sqrt((line1 * line1) + (line2 * line2) - 2.0 * line1 * line2 * cos(angle));
}






/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */







void received_command() {

	static float sup_angle2[25] = {0, };
	float fi[3] = {0, };

    /*============================================================*/


	/* Имеем разворот (классическое положение) */
	fi[0] = 3.1415926536;
	fi[1] = 3.1415926536;
	fi[2] = 3.1415926536;

	if(u_cmd[0].number_motor == 0) {
		if (u_cmd[0].op_cmd == MOVE) {
			angle[0] = u_cmd[0].angle_trg;
		}
		else if (u_cmd[0].op_cmd == HOLD) {
			angle[0] = angle[0];
		}
		else if (u_cmd[0].op_cmd == RELEASED) {
			angle[0] = angle[0];
		}
	}
	if(u_cmd[1].number_motor == 1) {
		if (u_cmd[1].op_cmd == MOVE) {
			angle[1] = u_cmd[1].angle_trg;
		}
		else if (u_cmd[1].op_cmd == HOLD) {
			angle[1] = angle[1];
		}
		else if (u_cmd[1].op_cmd == RELEASED) {
			angle[1] = angle[1];
		}
	}
	if(u_cmd[2].number_motor == 2) {
		if (u_cmd[2].op_cmd == MOVE) {
			angle[2] = u_cmd[2].angle_trg;
		}
		else if (u_cmd[2].op_cmd == HOLD) {
			angle[2] = angle[2];
		}
		else if (u_cmd[2].op_cmd == RELEASED) {
			angle[2] = angle[2];
		}
	}



//	float angle[3] = {
//			USB_Angle[0] | (USB_Angle[1] << 8) | (USB_Angle[2] << 8)  | (USB_Angle[3]  << 8),
//			USB_Angle[4] | (USB_Angle[5] << 8) | (USB_Angle[6] << 8)  | (USB_Angle[7]  << 8),
//			USB_Angle[8] | (USB_Angle[9] << 8) | (USB_Angle[10] << 8) | (USB_Angle[11] << 8)
//			USB_Angle[0] | (USB_Angle[1] << 8),
//			USB_Angle[2] | (USB_Angle[3] << 8),
//			USB_Angle[4] | (USB_Angle[5] << 8),
//	};
	angle[0] /= 1000;
	angle[1] /= 1000;
	angle[2] /= 1000;


    fi[0] += (angle[0] * 3.14) / 180.0;
    fi[1] += (angle[1] * 3.14) / 180.0;
    fi[2] += (angle[2] * 3.14) / 180.0;

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
    float l_p1p11 = lenPP(exoskeleton.p[p1], exoskeleton.p[p11]);
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

    float l_p9p11 = lenPP(exoskeleton.p[p9], exoskeleton.p[p11]);

    sup_angle2[20] = angle3Line(l_p4p11, l_p4p9, l_p9p11) - angle3Line(l_p10p11, l_p9p11, l_p9p10); /* ЗАВИСИМОСТЬ ОТ ЗНАКА +- */
                                                                                                    /* + ЕСЛИ ОТ НУЛЯ fi[0] */

    exoskeleton.p[p4].x         = exoskeleton.p[p9].x + l_p4p9 * cos((fi[0] - PI) + (fi[1] - PI) + sup_angle2[20]);
    exoskeleton.p[p4].y         = exoskeleton.p[p9].y + l_p4p9 * sin((fi[0] - PI) + (fi[1] - PI) + sup_angle2[20]);

    l_p3p4       = lenPP(exoskeleton.p[p3], exoskeleton.p[p4]);          /* Q1 */

    float l_p7p9 = lenPP(exoskeleton.p[p7], exoskeleton.p[p9]);

    if (l_p7p9 > (l_p8p9 + l_p7p8)) l_p7p9 = l_p8p9 + l_p7p8;



    sup_angle2[21] = angle3Line(l_p5p9, l_p7p9, l_p5p7) - angle3Line(l_p8p9, l_p7p8, l_p7p9);   /* ЗАВИСИМОСТЬ ОТ ЗНАКА +- */
                                                                                                /* + ЕСЛИ ОТ НУЛЯ fi[0] */

    exoskeleton.p[p5].x         = exoskeleton.p[p7].x + l_p5p7 * cos((fi[0] - PI) + (fi[1] - PI) + (fi[2] - PI) + sup_angle2[21]);
    exoskeleton.p[p5].y         = exoskeleton.p[p7].y + l_p5p7 * sin((fi[0] - PI) + (fi[1] - PI) + (fi[2] - PI) + sup_angle2[21]);

    l_p4p5          = lenPP(exoskeleton.p[p4], exoskeleton.p[p5]);       /* Q2 */


//    exoskeleton.current_q[0] = l_p2p3;
//    exoskeleton.current_q[1] = l_p3p4;
//    exoskeleton.current_q[2] = l_p4p5;

    /* Start */

    if (exoskeleton.current_q[0] < l_p2p3) {
    	motor[0].direction			= forward;
    }
    else motor[0].direction			= backforward;
    if (exoskeleton.current_q[1] < l_p3p4) {
    	motor[1].direction			= forward;
    }
    else motor[1].direction			= backforward;
    if (exoskeleton.current_q[2] < l_p4p5) {
    	motor[2].direction			= forward;
    }
    else motor[2].direction			= backforward;

	HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_RESET);

    HAL_TIM_Base_Start_IT(&htim4);

    if (exoskeleton.current_q[0] != l_p2p3) {
        motor[0].motor_en = true;
    }
	if (exoskeleton.current_q[1] != l_p3p4) {
	    motor[1].motor_en = true;
	}
	if (exoskeleton.current_q[2] != l_p4p5) {
	    motor[2].motor_en = true;
	}


	if ((u_cmd[0].speed == 0) || (u_cmd[1].speed == 0) || (u_cmd[2].speed == 0)) {
		CDC_Transmit_FS((uint8_t*)"SPD= 0", 6u);
	}
	else {
		motor[0].pwm_count_down 	= u_cmd[0].speed;
		motor[1].pwm_count_down 	= u_cmd[1].speed;
		motor[2].pwm_count_down 	= u_cmd[2].speed;
	}

    while(1) {

    	if (CMD_HDSP) {
    		CMD_HDSP = false;

    		motor[0].motor_en = false;
    		motor[1].motor_en = false;
    		motor[2].motor_en = false;

    		CDC_Transmit_FS((uint8_t*)"OK____", 6u);
    		break;
    	}
    	if (exoskeleton.current_q[0] <= u_lim[0].lim_min) {
    		HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
    		motor[0].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRLM0", 6u);
    	}
    	else if (exoskeleton.current_q[0] >= u_lim[0].lim_max) {
    		HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
    		motor[0].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRHM0", 6u);
    	}

    	if (exoskeleton.current_q[1] <= u_lim[1].lim_min) {
    		HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
    		motor[1].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRLM1", 6u);
    	}
    	else if (exoskeleton.current_q[1] >= u_lim[1].lim_max) {
    		HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
    		motor[1].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRHM1", 6u);
    	}

    	if (exoskeleton.current_q[2] <= u_lim[2].lim_min) {
    		HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);
    		motor[2].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRLM2", 6u);
    	}
    	else if (exoskeleton.current_q[2] >= u_lim[2].lim_max) {
    		HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
    		HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);
    		motor[2].motor_en = false;
    		CDC_Transmit_FS((uint8_t*)"ERRHM2", 6u);
    	}






//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//    	for (uint32_t i = 0; i < 100; i++) {}
//    	HAL_SPI_Receive_IT(&hspi1, &data, 1);
//    	for (uint32_t i = 0; i < 100; i++) {}
//    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    	//*********************//
//    	if (!(data & (1 << 7))) motor[0].enc_ch_state2		= true;
//    	else motor[0].enc_ch_state2							= false;
    	// The second sensor is EXTI handler sensor
    	//*********************//
//    	if (!(data & (1 << 6))) motor[1].enc_ch_state2		= true;
//    	else motor[1].enc_ch_state2							= false;
//    	if (!(data & (1 << 5))) motor[1].enc_ch_state1		= true;
//    	else motor[1].enc_ch_state1							= false;
    	//*********************//
//    	if (!(data & (1 << 4))) motor[2].enc_ch_state2		= true;
//    	else motor[2].enc_ch_state2							= false;
//    	if (!(data & (1 << 3))) motor[2].enc_ch_state1		= true;
//    	else motor[2].enc_ch_state1							= false;
    	//*********************//

    	if (motor[0].direction == forward) {
    		if (exoskeleton.current_q[0] >= l_p2p3) {
    			motor[0].motor_en = false;
    			HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
        	}
    	}
    	else if (motor[0].direction == backforward) {
    		if (exoskeleton.current_q[0] <= l_p2p3) {
    			motor[0].motor_en = false;
    			HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
        	}
    	}
    	if (motor[1].direction == forward) {
    		if (exoskeleton.current_q[1] >= l_p3p4) {
    			motor[1].motor_en = false;
    			HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
        	}
    	}
    	else if (motor[1].direction == backforward) {
    		if (exoskeleton.current_q[1] <= l_p3p4) {
    			motor[1].motor_en = false;
    			HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
        	}
    	}
    	if (motor[2].direction == forward) {
    		if (exoskeleton.current_q[2] >= l_p4p5) {
    			motor[2].motor_en = false;
    			HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);
        	}
    	}
    	else if (motor[2].direction == backforward) {
    		if (exoskeleton.current_q[2] <= l_p4p5) {
    			motor[2].motor_en = false;
    			HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
        		HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);
        	}
    	}

    	/* DISABLE */
    	if (!motor[0].motor_en && !motor[1].motor_en && !motor[2].motor_en) {
    		HAL_TIM_Base_Stop_IT(&htim4);
    		CDC_Transmit_FS((uint8_t*)"OK____", 6u);
    		break;
    	}

    }

	HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_RESET);

//	if ((motor[0].pid.trg_angle - motor[0].pid.current_angle) <= 0) {
//		  motor[0].motor_en				= false;
//		  HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
//	}

}


void home() {

	motor[0].direction			= backforward;
	motor[1].direction			= backforward;
	motor[2].direction			= backforward;

	motor[0].trg_up				= 10;
	motor[0].pwm_count_down 	= 500;
	motor[1].trg_up				= 10;
	motor[1].pwm_count_down 	= 500;
	motor[2].trg_up				= 10;
	motor[2].pwm_count_down 	= 500;

	HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim4);

    motor[0].motor_en = true;
    motor[1].motor_en = true;
    motor[2].motor_en = true;

    while(1) {

    	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) && motor[0].motor_en) {
    		motor[0].motor_en = false;
			HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
    	}
    	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) && motor[1].motor_en) {
			motor[1].motor_en = false;
			HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_RESET);
    	}
    	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET) && motor[2].motor_en) {
			motor[2].motor_en = false;
			HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_RESET);
    	}

    	if (!motor[0].motor_en && !motor[1].motor_en && !motor[2].motor_en) {

    		HAL_Delay(500);
    		/* First position */

    		motor[0].direction			= forward;
    		motor[1].direction			= forward;
    		motor[2].direction			= forward;

    	    motor[0].motor_en = true;
    	    motor[1].motor_en = true;
    	    motor[2].motor_en = true;

    	    HAL_Delay(800);

    		motor[0].motor_en = false;
    	    motor[1].motor_en = false;
    	    motor[2].motor_en = false;

			HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);

    	    HAL_TIM_Base_Stop_IT(&htim4);

    		/* First position */
    		exoskeleton.current_q[0] = 74.790899999999993;
    		exoskeleton.current_q[1] = 59.259999999999998;
    		exoskeleton.current_q[2] = 30.91;


    	    HAL_Delay(700);

			HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_RESET);

    		break;
    	}
    }
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* Motor 1 */
  motor[0].gpio1 				= GPIOB;
  motor[0].pin1					= GPIO_PIN_7;
  motor[0].gpio2 				= GPIOB;
  motor[0].pin2					= GPIO_PIN_6;
  motor[0].transfer_value		= 23.0;
  motor[0].direction			= forward;
  motor[0].motor_en				= false;
  motor[0].trg_up				= 10;
  motor[0].pwm_count_down 		= 800;
  /* Motor 2 */
  motor[1].gpio1 				= GPIOB;
  motor[1].pin1					= GPIO_PIN_5;
  motor[1].gpio2 				= GPIOB;
  motor[1].pin2					= GPIO_PIN_4;
  motor[1].transfer_value		= 23.0;
  motor[1].direction			= forward;
  motor[1].motor_en				= false;
  motor[1].trg_up				= 10;
  motor[1].pwm_count_down 		= 800;
  /* Motor 3 */
  motor[2].gpio1 				= GPIOB;
  motor[2].pin1					= GPIO_PIN_3;
  motor[2].gpio2 				= GPIOA;
  motor[2].pin2					= GPIO_PIN_15;
  motor[2].transfer_value		= 23.0;
  motor[2].direction			= forward;
  motor[2].motor_en				= false;
  motor[2].trg_up				= 10;
  motor[2].pwm_count_down 		= 800;

  /* First position */
  exoskeleton.current_q[0] = 74.790899999999993;
  exoskeleton.current_q[1] = 59.259999999999998;
  exoskeleton.current_q[2] = 30.91;

  /* Linear pitch value per screw rotation value */
  motor[0].linear_step_on_per = 0.5;
  motor[1].linear_step_on_per = 0.5;
  motor[2].linear_step_on_per = 0.5;


//  HAL_TIM_Base_Start_IT(&htim4);
//  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  HAL_Delay(1000);

  USB_Angle[0] = 0;
  USB_Angle[1] = 0;
  USB_Angle[2] = 0;

  u_lim[0].lim_max 	= 200.0;
  u_lim[0].lim_min 	= 0.0;
  u_lim[1].lim_max 	= 200.0;
  u_lim[1].lim_min 	= 0.0;
  u_lim[2].lim_max 	= 200.0;
  u_lim[2].lim_min 	= 0.0;

  u_cmd[0].speed	= 800;
  u_cmd[1].speed	= 800;
  u_cmd[2].speed	= 800;


//  received_command();



//  home();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  if (CMD_HOME) {
		  home();
		  CDC_Transmit_FS((uint8_t*)"OK____", 6u);
		  CMD_HOME = false;
	  }
	  else if (CMD_MAIN) {
		  received_command();
		  CDC_Transmit_FS((uint8_t*)"OK____", 6u);
		  CMD_MAIN = false;
	  }
	  else if (CMD_GET) {

		  uint8_t u = 0;
		  buffer_usb[u++] = 'T';
		  uint32_t tm = HAL_GetTick();
		  buffer_usb[u++] = ((tm & 0xFF000000) >> 24);
		  buffer_usb[u++] = ((tm & 0x00FF0000) >> 16);
		  buffer_usb[u++] = ((tm & 0x0000FF00) >> 8);
		  buffer_usb[u++] = ((tm & 0x000000FF) >> 0);
		  for (uint8_t i = 0; i < 3; i++) {
			  buffer_usb[u++] = 'M';
			  buffer_usb[u++] = i;
			  uint32_t agl = angle[0] * 1000;
			  buffer_usb[u++] = ((agl & 0xFF000000) >> 24);
			  buffer_usb[u++] = ((agl & 0x00FF0000) >> 16);
			  buffer_usb[u++] = ((agl & 0x0000FF00) >> 8);
			  buffer_usb[u++] = ((agl & 0x000000FF) >> 0);
			  uint32_t c_agl = exoskeleton.current_q[0] * 1000;
			  buffer_usb[u++] = ((c_agl & 0xFF000000) >> 24);
			  buffer_usb[u++] = ((c_agl & 0x00FF0000) >> 16);
			  buffer_usb[u++] = ((c_agl & 0x0000FF00) >> 8);
			  buffer_usb[u++] = ((c_agl & 0x000000FF) >> 0);
			  buffer_usb[u++] = u_cmd[i].op_cmd;
			  uint32_t lim_mn = u_lim[i].lim_min * 1000;
			  buffer_usb[u++] = ((lim_mn & 0xFF000000) >> 24);
			  buffer_usb[u++] = ((lim_mn & 0x00FF0000) >> 16);
			  buffer_usb[u++] = ((lim_mn & 0x0000FF00) >> 8);
			  buffer_usb[u++] = ((lim_mn & 0x000000FF) >> 0);
			  uint32_t lim_mx = u_lim[i].lim_max * 1000;
			  buffer_usb[u++] = ((lim_mx & 0xFF000000) >> 24);
			  buffer_usb[u++] = ((lim_mx & 0x00FF0000) >> 16);
			  buffer_usb[u++] = ((lim_mx & 0x0000FF00) >> 8);
			  buffer_usb[u++] = ((lim_mx & 0x000000FF) >> 0);
		  }
		  CDC_Transmit_FS((uint8_t*)buffer_usb, 59u);
		  CMD_GET = false;
	  }


//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//	  for (uint32_t i = 0; i < 100; i++) {}
//	  HAL_SPI_Receive_IT(&hspi1, &data, 1);
//	  for (uint32_t i = 0; i < 100; i++) {}
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//
//
//	  if (!(data & (1 << 7))) motor[0].enc_ch_state2		= true;
//	  else motor[0].enc_ch_state2							= false;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : M1_I2_Pin */
  GPIO_InitStruct.Pin = M1_I2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M1_I2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim4)	// 10000us
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
//				motor[i].pid.current_time++;
			}
		}
	}
	else if (htim == &htim3) {


	}

//	for(uint32_t i = 0; i < 1000; i++) {}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == GPIO_PIN_2) {
//		motor[0].motor_en = false;
//		HAL_GPIO_WritePin(motor[0].gpio1, motor[0].pin1, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(motor[0].gpio2, motor[0].pin2, GPIO_PIN_SET);
//	}
//	else if (GPIO_Pin == GPIO_PIN_10) {
//		motor[1].motor_en = false;
//		HAL_GPIO_WritePin(motor[1].gpio1, motor[1].pin1, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(motor[1].gpio2, motor[1].pin2, GPIO_PIN_SET);
//	}
//	else if (GPIO_Pin == GPIO_PIN_11) {
//		motor[2].motor_en = false;
//		HAL_GPIO_WritePin(motor[2].gpio1, motor[2].pin1, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(motor[2].gpio2, motor[2].pin2, GPIO_PIN_SET);
	}

	else if (GPIO_Pin == GPIO_PIN_9) {
		if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) {
			if (motor[0].direction == backforward) exoskeleton.current_q[0] -=
					motor[0].linear_step_on_per / (7.0 * motor[0].transfer_value);
		}
		else {
			if (motor[0].direction == forward) exoskeleton.current_q[0]	+=
					motor[0].linear_step_on_per / (7.0 * motor[0].transfer_value);
		}
	}

	else if (GPIO_Pin == GPIO_PIN_15) {
		if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) {
			if (motor[1].direction == backforward) exoskeleton.current_q[1]	-=
					motor[1].linear_step_on_per / (7.0 * motor[1].transfer_value);
		}
		else {
			if (motor[1].direction == forward) exoskeleton.current_q[1] +=
					motor[1].linear_step_on_per / (7.0 * motor[1].transfer_value);
		}
	}

	else if (GPIO_Pin == GPIO_PIN_13) {
		if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) {
			if (motor[2].direction == forward) exoskeleton.current_q[2]	+=
					motor[2].linear_step_on_per / (7.0 * motor[2].transfer_value);
		}
		else {
			if (motor[2].direction == backforward) exoskeleton.current_q[2]	-=
					motor[2].linear_step_on_per / (7.0 * motor[2].transfer_value);
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
