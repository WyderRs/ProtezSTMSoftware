/*
 * protocol.h
 *
 *  Created on: Sep 3, 2025
 *      Author: makar
 */

#ifndef PROTEZLIB_PROTOCOL_H_
#define PROTEZLIB_PROTOCOL_H_


#include "stm32f4xx_hal.h"
#include "stdbool.h"


/*Protocol codes*/
#define PR_PROTOCOL_CODE_SIDEPLATE		0x01
#define PR_PROTOCOL_CODE_WORKMODE		0x02
#define PR_PROTOCOL_CODE_CONFIG			0x03
#define PR_PROTOCOL_CODE_NUMMOTOR		0x04
#define PR_PROTOCOL_CODE_DIRECTION		0x05
#define PR_PROTOCOL_CODE_PWM			0x06
#define PR_PROTOCOL_CODE_TIME			0x07
#define PR_PROTOCOL_CODE_ANGLE			0x08
#define PR_PROTOCOL_CODE_SPEED			0x09
#define PR_PROTOCOL_CODE_DELAY			0x0A
#define PR_PROTOCOL_CODE_ADC			0x0B
#define PR_PROTOCOL_CODE_FEEDBACK		0x0C
#define PR_PROTOCOL_CODE_RUNNING		0x0D

/* Маска кода команды и маска данных */
#define PR_PROTOCOL_MASK_NUM_BYTES		0xE0
#define PR_PROTOCOL_MASK_CODE			0x1F


typedef struct ex_uart_t
{
	UART_HandleTypeDef* uart;				/* Структура uart */
	uint32_t			ex_counterData;		/* Количество данных uart на отправку */
	uint8_t				ex_data[200];		/* Массив uart */


	uint8_t 			RecvData[200];	/* Массив uart на прием*/
	uint8_t 			DataByte;		/* Переменная приема данных */
	uint32_t 			DataCount;		/* Счетчик данных приема */
	_Bool 				flag_Recv;		/* Флаг приема данных*/

} ex_uart_t;

/* Структура команды запуска */
typedef struct com_running_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_running_t;

/* Структура команды ацп */
typedef struct com_adc_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_adc_t;

/* Структура команды задержки */
typedef struct com_delay_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_delay_t;

/* Структура команды скорости */
typedef struct com_speed_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_speed_t;

/* Структура команды угла */
typedef struct com_angle_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_angle_t;

/* Структура команды времени работы */
typedef struct com_worktime_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_worktime_t;

/* Структура команды pwm */
typedef struct com_pwm_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_pwm_t;

/* Структура команды направления вращения вала двигателя */
typedef struct com_siderot_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_siderot_t;

/* Структура команды номера двигателя*/
typedef struct com_numotor_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_numotor_t;

/* Структура команды конфигурации */
typedef struct com_config_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_config_t;

/* Структура команды режима работы */
typedef struct com_workmode_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_workmode_t;

/* Структура команды определяющей сторону платы */
typedef struct com_sideplate_t
{
	uint8_t 			com;
	uint8_t 			value[10];
} com_sideplate_t;

/* Структура инструкции */
typedef struct instruction_t
{
	ex_uart_t			ex_uart;

	com_sideplate_t		com_sideplate;
	com_workmode_t		com_workmode;
	com_config_t		com_config;
	com_numotor_t		com_numotor;
	com_siderot_t		com_siderot;
	com_pwm_t			com_pwm;
	com_worktime_t		com_worktime;
	com_angle_t			com_angle;
	com_speed_t			com_speed;
	com_delay_t			com_delay;
	com_adc_t			com_adc;
	com_running_t		com_running;
} instruction_t;







/* Инициализация протокола обмена данными */
void protocol_Init();
/* Разбирает принятый пакет данных инструкции с uart */
void protocol_read_raw_uart(uint8_t* data, uint32_t cnt);
/* Разбирает принятый пакет данных инструкции */
void protocol_read_raw(uint8_t* data, uint32_t cnt);



#endif /* PROTEZLIB_PROTOCOL_H_ */
