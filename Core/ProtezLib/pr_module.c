/*
 * module.c
 *
 *  Created on: Sep 2, 2025
 *      Author: makar
 */


#include "pr_module.h"


/* externs */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern ADC_HandleTypeDef hadc1;

extern UART_HandleTypeDef huart6;

extern ex_uart_t uartStrc;
/* ****** global ****** */

/* Структура модуля */
module_t module;
/* Переменная времени в 0.01s (10ms) */
uint32_t common_timer_10ms;


/* Инициализирует структуру для работы с данными */
void module_Data_Init()
{
//	for (uint8_t i = 0; i < module.cntMotor; i++)
//	{
//		module.ctrl_data.ptr_adc[i] = i * module.ctrl_data.adc_size;
//		module.ctrl_data.ptr_enc[i] = i * module.ctrl_data.enc_size;
//	}
//	for (uint8_t i = 0; i < module.cntMotor; i++)
//	{
//		module.ctrl_data.ptr_pack_adc[i] = i * module.ctrl_data.adc_size;
//		module.ctrl_data.ptr_pack_enc[i] = (module.cntMotor + i) * module.ctrl_data.enc_size;
//	}


	for (uint8_t i = 0; i < module.cntMotor; i++)
	{
		module.ctrl_data.ptr_adc[i] = i * module.ctrl_data.adc_size;
		module.ctrl_data.ptr_enc[i] = i * module.ctrl_data.enc_size;
	}
	for (uint8_t i = 0; i < module.adc.cntMotorAdcEn; i++)
	{
		module.ctrl_data.ptr_pack_adc[i] = i * module.ctrl_data.adc_size;
		module.ctrl_data.ptr_pack_enc[i] = (module.adc.cntMotorAdcEn + i) * module.ctrl_data.enc_size;
	}

}

/* Сбрасывает компоненты энкодера */
void module_Enc_Reset()
{
	for (uint8_t i = 0; i < module.cntMotor; i++)
	{
		module.motor[i].enc.counter = 0;
	}
}

/* Останавливает работу АЦП */
void module_ADC_Stop()
{
	HAL_ADC_Stop(module.adc.tdf_adc);
	HAL_ADC_Stop_DMA(module.adc.tdf_adc);

	HAL_ADC_DeInit(module.adc.tdf_adc);

	HAL_TIM_Base_Stop_IT(module.adc.timADC);

	HAL_TIM_Base_DeInit(module.adc.timADC);

	module.adc.state = adc_disable;
}
/* Запускает работу АЦП */
void module_ADC_Start()
{
	HAL_TIM_Base_Start_IT(module.adc.timADC);
//	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(module.adc.tdf_adc, (uint32_t*)&module.ctrl_data.adc_data, module.ctrl_data.adc_size / 2);

	module.adc.state = adc_working;
}
/* Инициализирует таймер для АЦП */
void module_ADC_Timer_Init()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	module.adc.timADC->Instance = TIM2;
	module.adc.timADC->Init.Prescaler = 50 - 1;
	module.adc.timADC->Init.CounterMode = TIM_COUNTERMODE_UP;

	module.adc.timADC->Init.Period = (((HAL_RCC_GetSysClockFreq() / (module.adc.timADC->Init.Prescaler + 1)) / (module.ctrl_data.Points))) - 1;

	module.adc.timADC->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	module.adc.timADC->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(module.adc.timADC) != HAL_OK) Error_Handler();
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(module.adc.timADC, &sClockSourceConfig) != HAL_OK) Error_Handler();
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(module.adc.timADC, &sMasterConfig) != HAL_OK) Error_Handler();


//	htim2.Init.Prescaler = 50 - 1;
//	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//
//	htim2.Init.Period = (((HAL_RCC_GetSysClockFreq() / (htim2.Init.Prescaler + 1)) / (module.ctrl_data.Points))) - 1;
//
//	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
//	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
//	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
//	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

}

/* Инициализация АЦП */
void module_ADC_Init()
{
	module.adc.cntMotorAdcEn = 0;
	for (uint8_t i = 0; i < module.cntMotor; i++) {
		if (module.motor[i].adc.adc_en) module.adc.cntMotorAdcEn++;
	}

	module.ctrl_data.adc_size = SIZEPACK * module.adc.cntMotorAdcEn;
	module.ctrl_data.enc_size = SIZEPACK * module.adc.cntMotorAdcEn;

	/*Initialization ADC*/
	ADC_ChannelConfTypeDef sConfig = {0};
	module.adc.tdf_adc->Instance = ADC1;
	module.adc.tdf_adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	module.adc.tdf_adc->Init.Resolution = ADC_RESOLUTION_12B;
	module.adc.tdf_adc->Init.ScanConvMode = ENABLE;
	module.adc.tdf_adc->Init.ContinuousConvMode = DISABLE;
	module.adc.tdf_adc->Init.DiscontinuousConvMode = DISABLE;
	module.adc.tdf_adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	module.adc.tdf_adc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	module.adc.tdf_adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	module.adc.tdf_adc->Init.NbrOfConversion = module.adc.cntMotorAdcEn;
	module.adc.tdf_adc->Init.DMAContinuousRequests = ENABLE;
	module.adc.tdf_adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(module.adc.tdf_adc) != HAL_OK)
	{
		Error_Handler();
	}
	/*------------------*/
	uint8_t rank = 1;

	for (uint8_t i = 0; i < module.cntMotor; i++)
	{
		if (module.motor[i].adc.adc_en)
		{
			sConfig.Channel = module.motor[i].adc.adc_ch;
			sConfig.Rank = rank;
			sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
			if (HAL_ADC_ConfigChannel(module.adc.tdf_adc, &sConfig) != HAL_OK) Error_Handler();
			rank++;
		}
	}
	module_ADC_Timer_Init();

	module.adc.state = adc_config;
}

/* Останавливает работу двигателя
 * mtr_t 	- структура двигателя
 */
void module_motor_Stop(motor_t* mtr_t)
{
	HAL_TIM_PWM_Stop_IT(mtr_t->timer.tim, mtr_t->timer.ch[0]);
	HAL_TIM_PWM_Stop_IT(mtr_t->timer.tim, mtr_t->timer.ch[1]);

	mtr_t->state = finished;

	*mtr_t->timer.CCR[0] = 0;
	*mtr_t->timer.CCR[1] = 0;

	mtr_t->time.begin_wt = 0;
	mtr_t->time.end_wt = 0;
	mtr_t->time.corrent_wt = 0;
}

/* Запускает работу двигателя
 * mtr_t 	- структура двигателя
 */
void module_motor_Start(motor_t* mtr_t)
{
	HAL_TIM_PWM_Start_IT(mtr_t->timer.tim, mtr_t->timer.ch[0]);
	HAL_TIM_PWM_Start_IT(mtr_t->timer.tim, mtr_t->timer.ch[1]);

	mtr_t->state = working;
}

/* Инициализирует канал АЦП двигателя
 * mtr_t	- структура двигателя
 * ср		- канал ацп
 */
void module_setADCParam(motor_t* mtr_t, adc_ch_t ch)
{
	mtr_t->adc.adc_ch = ch;
}

/* Инициализирует энкодер двигателя
 * mtr_t  	- структура двигателя
 * port_1 	- порт вывода энкодера
 * pin_1  	- пин вывода энкодера
 * port_2 	- порт вывода энкодера
 * pin_2  	- пин вывода энкодера
 * pos 	  	- выбор вывода как положительно-направленное
 * 		true: exti_p - положительное направление, инеаче read_p
 */
void module_setEncParam(motor_t* mtr_t, GPIO_TypeDef* port_1, uint16_t pin_1, GPIO_TypeDef* port_2, uint16_t pin_2, _Bool pos)
{
	mtr_t->enc.exti_p.port = port_1;
	mtr_t->enc.exti_p.pin = pin_1;

	mtr_t->enc.read_p.port = port_2;
	mtr_t->enc.read_p.pin = pin_2;

	if (pos) mtr_t->enc.index = forward;
	else mtr_t->enc.index = backforward;

}
/* Инициализирует таймер двигателя
 * mtr_t - структура двигателя
 * tim	 - указатель на таймер
 * ch1   - канал двигателя
 * ch2 	 - канал двигателя
 * dir	 - инвертирование направления
 */
void module_setTimParam(motor_t* mtr_t, TIM_HandleTypeDef* tim, uint32_t ch1, uint32_t ch2, _Bool dir)
{
	mtr_t->timer.tim = tim;

	uint32_t ch[2] = {0, };
	if (dir) {
		ch[0] = ch2;
		ch[1] = ch1;
	}
	else {
		ch[0] = ch1;
		ch[1] = ch2;
	}
	mtr_t->timer.ch[0] = ch[0];
	mtr_t->timer.ch[1] = ch[1];

	for (uint8_t i = 0; i < 2; i++)
	{
		if (ch[i] == TIM_CHANNEL_1) {
			mtr_t->timer.CCR[i] = &(mtr_t->timer.tim->Instance->CCR1);
		}
		else if (ch[i] == TIM_CHANNEL_2) {
			mtr_t->timer.CCR[i] = &(mtr_t->timer.tim->Instance->CCR2);
		}
		else if (ch[i] == TIM_CHANNEL_3) {
			mtr_t->timer.CCR[i] = &(mtr_t->timer.tim->Instance->CCR3);
		}
		else if (ch[i] == TIM_CHANNEL_4) {
			mtr_t->timer.CCR[i] = &(mtr_t->timer.tim->Instance->CCR4);
		}
	}
}

/* Инициализация модуля
 * Возвращает: module_state - статус модуля
 */
module_state module_Init()
{
	/* Инициализация модуля */
	module.state = mod_config;
	module.cntMotor = 6;
	module.ctrl_data.Points = 500;
	module.adc.timADC = &htim2;
	module.adc.tdf_adc = &hadc1;
	/* Инициализация протокола передачи данных */
	protocol_Init(&huart6);
	/* Инициализация двигателей */
	module_setTimParam(&module.motor[0], &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, true);
	module_setEncParam(&module.motor[0], ENC_GPIO_2, ENC_PIN_2, SUP_ENC_GPIO_2, SUP_ENC_PIN_2, false);
	module_setADCParam(&module.motor[0], ADC_Channel_5);

	module_setTimParam(&module.motor[1], &htim3, TIM_CHANNEL_3, TIM_CHANNEL_4 , false);
	module_setEncParam(&module.motor[1], ENC_GPIO_1, ENC_PIN_1, SUP_ENC_GPIO_1, SUP_ENC_PIN_1, true);
	module_setADCParam(&module.motor[1], ADC_Channel_3);

	module_setTimParam(&module.motor[2], &htim4, TIM_CHANNEL_3, TIM_CHANNEL_4, false);
	module_setEncParam(&module.motor[2], ENC_GPIO_3, ENC_PIN_3, SUP_ENC_GPIO_3, SUP_ENC_PIN_3, false);
	module_setADCParam(&module.motor[2], ADC_Channel_4);

	module_setTimParam(&module.motor[3], &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
	module_setEncParam(&module.motor[3], ENC_GPIO_4, ENC_PIN_4, SUP_ENC_GPIO_4, SUP_ENC_PIN_4, false);
	module_setADCParam(&module.motor[3], ADC_Channel_2);

	module_setTimParam(&module.motor[4], &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
	module_setEncParam(&module.motor[4], ENC_GPIO_5, ENC_PIN_5, SUP_ENC_GPIO_5, SUP_ENC_PIN_5, false);
	module_setADCParam(&module.motor[4], ADC_Channel_7);

	module_setTimParam(&module.motor[5], &htim5, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
	module_setEncParam(&module.motor[5], ENC_GPIO_6, ENC_PIN_6, SUP_ENC_GPIO_6, SUP_ENC_PIN_6, false);
	module_setADCParam(&module.motor[5], ADC_Channel_6);

	/* Запуск служебных функций */

	HAL_TIM_Base_Start_IT(&htim11);		/* Обработка состояний двигателей */
	HAL_TIM_Base_Start_IT(&htim10);		/* Общий системный таймер */
	HAL_TIM_Base_Start_IT(&htim9);		/* Таймер для формирования и отправки пакета данных */

//	HAL_UART_Receive_IT(&huart6, (uint8_t*)&uartStrc.DataByte, 1);
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)&uartStrc.DataByte, 1);

	module.state = mod_ready;
	return 0;
}
/* Применяет принятые команды для двигателя */
void module_motor_SetParam(instruction_t* instruction)
{
	/*if (module.state == mod_working) return;*/

	uint8_t num = instruction->com_numotor.value[0];

	if (instruction->com_running.value[0] == 1)
	{
//		if (module.motor[num].state == config) module.motor[num].state = started;

		if (module.state != mod_working)
		{
			if (module.motor[num].state == config)
			{
				module.state = mod_start_work;
			}
		}
		return;
	}


	module.motor[num].time.wt = (instruction->com_worktime.value[0] | (instruction->com_worktime.value[1] << 8)) * 100;
	module.motor[num].time.delay_wt = (instruction->com_delay.value[0] | (instruction->com_delay.value[1] << 8)) * 100;
	module.motor[num].dirot = instruction->com_siderot.value[0];
	module_motor_SetPwm(&module.motor[num], instruction->com_pwm.value[0] * 10);
	module.motor[num].adc.adc_en = instruction->com_adc.value[0];

	if (module.motor[num].time.wt != 0) module.motor[num].state = config;
}

/* Устанавливает время работы и задержку перед началом работы
 * mtr_t 	- Структура двигателя
 */
void module_motor_SetTime(motor_t* mtr_t)
{
	mtr_t->time.corrent_wt = common_timer_10ms;
	mtr_t->time.begin_wt = common_timer_10ms + mtr_t->time.delay_wt;
	mtr_t->time.end_wt = mtr_t->time.begin_wt + mtr_t->time.wt;

	module.cntMotorWorking++;
	mtr_t->state = started;
}
/* Устанавливает шим
 * mtr_t 	- Структура двигателя
 * pwm 		- Значение шим
 */
void module_motor_SetPwm(motor_t* mtr_t, uint32_t pwm)
{
	if (pwm >= 1000) pwm = 1000;

	if (mtr_t->dirot == stop)
	{
		*mtr_t->timer.CCR[0] = 0;
		*mtr_t->timer.CCR[1] = 0;
	}
	else if (mtr_t->dirot == left)
	{
		*mtr_t->timer.CCR[0] = pwm;
		*mtr_t->timer.CCR[1] = 0;
	}
	else if (mtr_t->dirot == right)
	{
		*mtr_t->timer.CCR[0] = 0;
		*mtr_t->timer.CCR[1] = pwm;
	}
	else if (mtr_t->dirot == hold)
	{
		*mtr_t->timer.CCR[0] = pwm;
		*mtr_t->timer.CCR[1] = pwm;
	}
	else return;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if 		(htim->Instance == TIM1)	{}
	else if (htim->Instance == TIM3)	{}
	else if (htim->Instance == TIM4)	{}
	else if (htim->Instance == TIM5)	{}
	else if (htim->Instance == TIM2)
	{
		for (uint8_t i = 0; i < module.cntMotor; i++)
		{
			if (module.motor[i].adc.adc_en)
			{
				module.ctrl_data.enc_data[module.ctrl_data.ptr_enc[i] + module.motor[i].enc_counter++] =
						(module.motor[i].enc.counter & 0x00ff);
				module.ctrl_data.enc_data[module.ctrl_data.ptr_enc[i] + module.motor[i].enc_counter++] =
						((module.motor[i].enc.counter & 0xff00) >> 8);
			}
		}
	}
	else if (htim->Instance == TIM9)
	{
		if (module.flag_mesCplt)
		{
			uint8_t w = 0;
			/* Упаковываем данные АЦП */
			for (uint8_t i = 0; i < module.cntMotor; i++)
			{
				if (module.motor[i].adc.adc_en)
				{
					for (uint32_t j = w * module.ctrl_data.adc_size; j < (w + 1) * module.ctrl_data.adc_size; j++)
					{
						module.ctrl_data.PackAll[module.ctrl_data.ptr_pack_adc[w] + j] = module.ctrl_data.adc_data[j];
					}
					w++;
				}
			}
			/* Упаковываем данные энкодера*/
			for (uint8_t i = 0; i < module.cntMotor; i++)
			{
				if (module.motor[i].adc.adc_en)
				{
					for (uint32_t j = w * module.ctrl_data.enc_size; j < (w + 1) * module.ctrl_data.adc_size; j++)
					{
						module.ctrl_data.PackAll[/*module.ctrl_data.ptr_pack_enc[w - 1]*/ + j] =
								module.ctrl_data.enc_data[j - w * module.ctrl_data.enc_size];
					}
				}
				module.motor[i].enc_counter = 0;
			}
			/* Упаковываем еще ... */
			w = 0;
			module.flag_mesCplt = false;

			/* Отправляем пакет данных */

		}
	}
	else if (htim->Instance == TIM10)
	{
		common_timer_10ms++;

		for (uint8_t i = 0; i < module.cntMotor; i++)
		{
			if ((module.motor[i].state == started) || (module.motor[i].state == working))
			{
				module.motor[i].time.corrent_wt += 1;
			}
		}
	}
	else if (htim->Instance == TIM11)
	{
		/* Модуль начал работу (выполнение инструкции) */
		if (module.state == mod_start_work)
		{
			/* Инициализация ацп */
			module_ADC_Init();
			/* Инициализация структуры работы с данными */
			module_Data_Init();
			/* Сброс энкодеров */
			module_Enc_Reset();


			module.cntMotorWorking = 0;
			for (uint8_t i = 0; i < module.cntMotor; i++)
			{
				if (module.motor[i].state == config) module_motor_SetTime(&module.motor[i]);
			}

			DRIVER_CTRL_ON;
			module.state = mod_working;
		}
		/* Модуль в процессе работы (исполнения инструкции) */
		if (module.state == mod_working)
		{
			if (module.adc.state == adc_config)
			{
				module_ADC_Start();
			}
			for (uint8_t i = 0; i < module.cntMotor; i++)
			{
				if (module.motor[i].state == started)
				{
					if (module.motor[i].time.begin_wt <= module.motor[i].time.corrent_wt)
					{
						module_motor_Start(&module.motor[i]);
					}
				}
				else if (module.motor[i].state == working)
				{
					/* Здесь можно реализовать чтение ацп и отправку на тыльную строну данных для установки шим*/
					if (module.motor[i].time.end_wt <= module.motor[i].time.corrent_wt)
					{
						module_motor_Stop(&module.motor[i]);
					}
				}
			}
			/* Модуль проверяет статус всех двигателей и заканчивает работу (выполнение инструкции) */
			for (uint8_t i = 0; i < module.cntMotor; i++)
			{
				if (module.motor[i].state == finished)
				{
					module.cntMotorWorking--;
					module.motor[i].state = no_config;
				}
				if (module.cntMotorWorking == 0)
				{
					module.state = mod_ending_work;
					break;
				}
			}
		}
		/* Модуль закончил работу (выполнять инструкцию) */
		if (module.state == mod_ending_work)
		{
			DRIVER_CTRL_OFF;

			if (module.adc.state == adc_working) module_ADC_Stop();

			module.state = mod_ready;
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	for (uint8_t i = 0; i < module.cntMotor; i++)
	{
		if (module.motor[i].enc.exti_p.pin == GPIO_Pin)
		{
			/* Если направление вращения энкодера совпадает с установленным (правильном) */
			if (HAL_GPIO_ReadPin(module.motor[i].enc.read_p.port, module.motor[i].enc.read_p.pin) == (GPIO_PinState)module.motor[i].enc.index)
			{
				module.motor[i].enc.counter++;
				break;
			}
			/* Если направление вращения энкодера НЕ совпадает с установленным (правильном) */
			else
			{

			}
		}
	}
	/*************************************************************************************************/
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	module.flag_mesCplt = true;

//	for (uint8_t i = 0; i < module.cntMotor; i++)
//	{
//		if (module.motor[i].adc.adc_en)
//		{
//			module.motor[i].enc_counter = 0;
//		}
//	}
}







