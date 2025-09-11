/*
 * module.h
 *
 *  Created on: Sep 2, 2025
 *      Author: makar
 */

#ifndef PROTEZLIB_PR_MODULE_H_
#define PROTEZLIB_PR_MODULE_H_

#include "protocol.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"

/* Defines */
#define ENC_GPIO_1					GPIOB
#define ENC_PIN_1					GPIO_PIN_10
#define ENC_GPIO_2					GPIOB
#define ENC_PIN_2					GPIO_PIN_14
#define ENC_GPIO_3					GPIOB
#define ENC_PIN_3					GPIO_PIN_13
#define ENC_GPIO_4					GPIOC
#define ENC_PIN_4					GPIO_PIN_5
#define ENC_GPIO_5					GPIOC
#define ENC_PIN_5					GPIO_PIN_3
#define ENC_GPIO_6					GPIOC
#define ENC_PIN_6					GPIO_PIN_1

#define SUP_ENC_GPIO_1				GPIOB
#define SUP_ENC_PIN_1				GPIO_PIN_2
#define SUP_ENC_GPIO_2				GPIOB
#define SUP_ENC_PIN_2				GPIO_PIN_15
#define SUP_ENC_GPIO_3				GPIOB
#define SUP_ENC_PIN_3				GPIO_PIN_12
#define SUP_ENC_GPIO_4				GPIOC
#define SUP_ENC_PIN_4				GPIO_PIN_4
#define SUP_ENC_GPIO_5				GPIOC
#define SUP_ENC_PIN_5				GPIO_PIN_2
#define SUP_ENC_GPIO_6				GPIOC
#define SUP_ENC_PIN_6				GPIO_PIN_0

#define DRIVER_CTRL_GPIO			GPIOA
#define DRIVER_CTRL_PIN				GPIO_PIN_15

#define DRIVER_CTRL_ON				HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_SET)
#define DRIVER_CTRL_OFF				HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_RESET)

#define SIZEPACK					40

/* Cтатус АЦП */
typedef enum adc_state_t
{
	adc_disable = 0,
	adc_config = 1,
	adc_working = 2,
} adc_state_t;

/* Структура АЦП */
typedef struct adc_root_t
{
	ADC_HandleTypeDef*	 	tdf_adc;
	TIM_HandleTypeDef*		timADC;
	uint8_t					cntMotorAdcEn;		/* Количество двигателей с вклююченным ацп */
	adc_state_t				state;
} adc_root_t;

/* Структура работы с данными */
typedef struct ctrl_data_t
{
	uint32_t 				Points;				/* Количество измеряемых данных за 1с */

	uint8_t 				adc_data[500];		/* Массив с данными АЦП */
	uint8_t 				enc_data[500];		/* Массив с данными энкодера*/

	uint32_t				adc_size;			/* Количество данных ацп */
	uint32_t				enc_size;			/* Количество данных энкодера */

	/*******/
	uint32_t				ptr_adc[6];			/* Места в массиве ацп для распределения данных */
	uint32_t				ptr_enc[6];			/* Места в массиве энкодера для распределения данных */


	uint32_t				ptr_pack_adc[6];	/* Места в общем массиве для распределения данных ацп */
	uint32_t				ptr_pack_enc[6];	/* Места в общем массиве для распределения данных энкодера */
	/*******/
	uint32_t				PackAll[2000];		/* Массив содержащий все данные */
	uint32_t				PackSize;			/* Количество данных в пакете */
} ctrl_data_t;

/* Направление двигателя */
typedef enum m_direction_t
{
	stop = 0,
	left,
	right,
	hold,
} m_direction_t;

/* Статус двигателя */
typedef enum m_state_t
{
	no_config = 0,
	config,
	started,
	working,
	finished,
} m_state_t;

/* Структура для работы с временными характеристиками двигателя */
typedef struct basetime_t
{
	uint32_t 				wt;				// wt - work time
	uint32_t 				begin_wt;
	uint32_t 				corrent_wt;
	uint32_t 				end_wt;
	uint32_t 				delay_wt;

} basetime_t;

/* Тип направления */
typedef enum dirIndex_t
{
	forward = false,
	backforward = true,
} dirIndex_t;

/* Статус модуля */
typedef enum module_state
{
	mod_ready			= 0,
	mod_start_work		= 1,
	mod_working			= 2,
	mod_ending_work		= 3,
	mod_config			= 4,
} module_state;

/* Список возможных каналов АЦП */
typedef enum adc_ch_t {
	ADC_Channel_1 = ADC_CHANNEL_1, ADC_Channel_2 = ADC_CHANNEL_2,
	ADC_Channel_3 = ADC_CHANNEL_3, ADC_Channel_4 = ADC_CHANNEL_4,
	ADC_Channel_5 = ADC_CHANNEL_5, ADC_Channel_6 = ADC_CHANNEL_6,
	ADC_Channel_7 = ADC_CHANNEL_7, ADC_Channel_8 = ADC_CHANNEL_8,
	ADC_Channel_9 = ADC_CHANNEL_9, ADC_Channel_10 = ADC_CHANNEL_10,
	ADC_Channel_11 = ADC_CHANNEL_11, ADC_Channel_12 = ADC_CHANNEL_12,
} adc_ch_t;

typedef struct adc_t
{
	adc_ch_t				adc_ch;
	_Bool					adc_en;
} adc_t;
/*Структура порта ввода-вывода*/
typedef struct gpio_t
{
	GPIO_TypeDef*			 port;
	uint16_t				 pin;
} gpio_t;

/* Структура энкодера
 * exti_p - пин-прерывания
 * read_p - пин-чтения
 */
typedef struct encoder_t
{
	gpio_t					exti_p;
	gpio_t					read_p;
	dirIndex_t				index;
	uint16_t				counter;
} encoder_t;

/* Структура таймера двигателя */
typedef struct m_timer_t
{
	TIM_HandleTypeDef*		tim;
	uint32_t 				ch[2];
	volatile uint32_t*		CCR[2];
} m_timer_t;

/* Структура двигателя */
typedef struct motor_t
{
	adc_t					adc;
	m_timer_t 				timer;
	encoder_t				enc;
	basetime_t				time;
	m_direction_t			dirot;
	m_state_t				state;

	/* Служебные переменные */
	uint32_t 				enc_counter;
} motor_t;

/* Структура модуля */
typedef struct module_t
{
	ctrl_data_t				ctrl_data;				/* Структура для работы с данными */
	adc_root_t				adc;					/* Структура для работы с ацп */
	uint8_t					cntMotor;				/* Количество двигателей */
	uint8_t					cntMotorWorking;		/* Количество работающих двигателей */
	module_state			state;					/* Статус модуля */
	motor_t 				motor[6];				/* Массив структур двигателей */
	_Bool					uartRecvCom;			/* Флаг управления (устанавливается если устройство получает данные по uart */
	_Bool					flag_mesCplt;			/* Флаг завершение цикла приема пакетов с DMA */
} module_t;




/******************************PROTOTYPES*******************************/
/* Инициализация модуля */
module_state module_Init();

/* Инициализирует структуру для работы с данными */
void module_Data_Init();
/* Сбрасывает компоненты энкодера */
void module_Enc_Reset();
/* Останавливает работу АЦП */
void module_ADC_Stop();
/* Запускает работу АЦП */
void module_ADC_Start();
/* Инициализирует таймер для АЦП */
void module_ADC_Timer_Init();
/* Инициализация АЦП */
void module_ADC_Init();
/* Устанавливает время работы и задержку перед началом работы */
void module_motor_SetTime(motor_t* mtr_t);
/* Устанавливает шим */
void module_motor_SetPwm(motor_t* mtr_t, uint32_t pwm);
/* Применяет принятые команды для двигателя */
void module_motor_SetParam(instruction_t* instruction);
/* Останавливает работу двигателя */
void module_motor_Stop(motor_t* mtr_t);
/* Запускает работу двигателя */
void module_motor_Start(motor_t* mtr_t);
/* Инициализирует канал АЦП двигателя */
void module_setADCParam(motor_t* mtr_t, adc_ch_t ch);
/* Инициализирует таймер двигателя */
void module_setTimParam(motor_t* mtr_t, TIM_HandleTypeDef* tim, uint32_t ch1, uint32_t ch2, _Bool dir);
/* Иницализирует энкодер двигателя */
void module_setEncParam(motor_t* mtr_t, GPIO_TypeDef* port_1, uint16_t pin_1, GPIO_TypeDef* port_2, uint16_t pin_2, _Bool pos);
/***********************************************************************/

















#endif /* PROTEZLIB_PR_MODULE_H_ */
