/*
 * protocol.c
 *
 *  Created on: Sep 3, 2025
 *      Author: makar
 */


#include "protocol.h"
#include "pr_module.h"

/* Маркеры отвечающие за опредередение данных с ПК */
uint8_t PR_PROTOCOL_START_PACK[2] = {0xEA, 0xBC};
uint8_t PR_PROTOCOL_STOP_PACK[2] = {0xBC, 0xAE};
/* Маркеры отвечающие за определение данных двигателя (селекцию) */
uint8_t PR_PROTOCOL_START[2] = {0xDE, 0xAD};
uint8_t PR_PROTOCOL_STOP[2] = {0xBE, 0xEF};
/* Маркеры отправки данных команды по uart */
uint8_t PR_PROTOCOL_UART_START[2]	= {0xAA, 0xBB};
uint8_t PR_PROTOCOL_UART_STOP[2]	= {0xCC, 0xDD};
/* Маркеры отвечающие за определение данных по uart*/
uint8_t PR_PROTOCOL_PACK_DATA_START[2] = {0xEE, 0xDD};
uint8_t PR_PROTOCOL_PACK_DATA_STOP[2] = {0xCC, 0xBB};
/* Переменные для работы с uart */
ex_uart_t uartStrc;					/* Структура работы с Uart */
m_direction_t trg_side = right;		/* Направление (правильное) двигателей */

/* Инициализация протокола обмена данными
 *	uart_typedef	- структура uart для обмена данными со вторым модулем
 */
void protocol_Init(UART_HandleTypeDef* uart_typedef)
{
	uartStrc.uart = uart_typedef;
}

/* Разбирает принятый пакет данных инструкции
 * data	- массив данных
 * cnt	- количество байт данных
 */
void protocol_read_raw(uint8_t* data, uint32_t cnt)
{
	if (cnt == 0) return;

	uint8_t last_x = 0;
	_Bool flag = false;

	uint8_t sub_pack_cnt = 0;
	uint32_t temp_cnt = 0;

	uint8_t sub_cmd[6][30] = {{0, },};
	uint32_t sub_cmd_cnt[6] = {0, };

	for (uint32_t i = 0; i < cnt; i++)
	{
		if ((data[i] == PR_PROTOCOL_STOP[1]) && (last_x == PR_PROTOCOL_STOP[0]))
		{
			sub_cmd[sub_pack_cnt][temp_cnt - 1] = 0;
			sub_cmd_cnt[sub_pack_cnt] = temp_cnt;
			sub_pack_cnt++;
			flag = false;
		}
		else if (flag)
		{
			sub_cmd[sub_pack_cnt][temp_cnt] = data[i];
			temp_cnt++;
		}
		else if ((data[i] == PR_PROTOCOL_START[1]) && (last_x == PR_PROTOCOL_START[0]))
		{
			flag = true;
			temp_cnt = 0;
		}
		last_x = data[i];
	}
	/* Формируем маркер данных */
	uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_UART_START[0];
	uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_UART_START[1];

	for (uint8_t i = 0; i < sub_pack_cnt; i++)
	{
		instruction_t tepm_instr = {0};
		m_direction_t temp_trg_side = left;
		/* Формируем маркер данных двигателя */
		uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_START[0];
		uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_START[1];

		for (uint16_t j = 0; j < sub_cmd_cnt[i];)
		{
			uint8_t header = sub_cmd[i][j];
			uint8_t code = header & PR_PROTOCOL_MASK_CODE;
			uint8_t numBytes = (header & PR_PROTOCOL_MASK_NUM_BYTES) >> 5;

			j++;

			if (j + numBytes > sub_cmd_cnt[i]) {
				break;
			}
			switch (code) {
			case PR_PROTOCOL_CODE_SIDEPLATE:
				tepm_instr.com_sideplate.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_sideplate.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_sideplate.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_WORKMODE:
				tepm_instr.com_workmode.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_workmode.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_workmode.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_CONFIG:
				tepm_instr.com_config.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_config.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_config.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_NUMMOTOR:
				tepm_instr.com_numotor.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_numotor.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_numotor.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_DIRECTION:
				tepm_instr.com_siderot.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_siderot.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_siderot.value[idx];
					}
				}
				temp_trg_side = uartStrc.ex_data[uartStrc.ex_counterData - 1];

				break;
			case PR_PROTOCOL_CODE_PWM:
				tepm_instr.com_pwm.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_pwm.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_pwm.value[idx];
					}
				}
				if (trg_side == temp_trg_side)
				{
					tepm_instr.com_pwm.value[0] = 10;
				}
				else if (temp_trg_side == left)
				{
					uartStrc.ex_data[uartStrc.ex_counterData - 1] = 10;
				}
				break;
			case PR_PROTOCOL_CODE_TIME:
				tepm_instr.com_worktime.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_worktime.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_worktime.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_ANGLE:
				tepm_instr.com_angle.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_angle.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_angle.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_SPEED:
				tepm_instr.com_speed.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_speed.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_speed.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_DELAY:
				tepm_instr.com_delay.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_delay.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_delay.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_ADC:
				tepm_instr.com_adc.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_adc.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_adc.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			case PR_PROTOCOL_CODE_RUNNING:
				tepm_instr.com_running.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_running.value[idx] = sub_cmd[i][j++];
					{
						uartStrc.ex_data[uartStrc.ex_counterData++] = tepm_instr.com_running.value[idx];
					}
				}
				//				uartStrc.ex_data[uartStrc.ex_counterData - 2];
				//				uartStrc.ex_data[uartStrc.ex_counterData - 1];
				break;
			default:
				// Неизвестный код: пропускаем данные
				j += numBytes;
				break;
			}
		}
		module_motor_SetParam(&tepm_instr);
		/* Формируем маркер данных двигателя */
		uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_STOP[0];
		uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_STOP[1];
	}
	/* Формируем маркер данных */
	uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_UART_STOP[0];
	uartStrc.ex_data[uartStrc.ex_counterData++] = PR_PROTOCOL_UART_STOP[1];

	HAL_UART_Transmit_IT(uartStrc.uart, uartStrc.ex_data, uartStrc.ex_counterData);
	uartStrc.ex_counterData = 0;
}
/* Разбирает принятый пакет данных инструкции с uart
 * data	- массив данных
 * cnt	- количество байт данных
 */
void protocol_read_raw_uart(uint8_t* data, uint32_t cnt)
{
	if (cnt == 0) return;

	uint8_t last_x = 0;
	_Bool flag = false;

	uint8_t sub_pack_cnt = 0;
	uint32_t temp_cnt = 0;

	uint8_t sub_cmd[6][30] = {{0, },};
	uint32_t sub_cmd_cnt[6] = {0, };

	for (uint32_t i = 0; i < cnt; i++)
	{
		if ((data[i] == PR_PROTOCOL_STOP[1]) && (last_x == PR_PROTOCOL_STOP[0]))
		{
			sub_cmd[sub_pack_cnt][temp_cnt - 1] = 0;
			sub_cmd_cnt[sub_pack_cnt] = temp_cnt;
			sub_pack_cnt++;
			flag = false;
		}
		else if (flag)
		{
			sub_cmd[sub_pack_cnt][temp_cnt] = data[i];
			temp_cnt++;
		}
		else if ((data[i] == PR_PROTOCOL_START[1]) && (last_x == PR_PROTOCOL_START[0]))
		{
			flag = true;
			temp_cnt = 0;
		}
		last_x = data[i];
	}

	for (uint8_t i = 0; i < sub_pack_cnt; i++)
	{
		instruction_t tepm_instr = {0};

		for (uint16_t j = 0; j < sub_cmd_cnt[i];)
		{
			uint8_t header = sub_cmd[i][j];
			uint8_t code = header & PR_PROTOCOL_MASK_CODE;
			uint8_t numBytes = (header & PR_PROTOCOL_MASK_NUM_BYTES) >> 5;

			j++;

			if (j + numBytes > sub_cmd_cnt[i]) {
				break;
			}
			switch (code) {
			case PR_PROTOCOL_CODE_SIDEPLATE:
				tepm_instr.com_sideplate.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_sideplate.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_WORKMODE:
				tepm_instr.com_workmode.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_workmode.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_CONFIG:
				tepm_instr.com_config.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_config.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_NUMMOTOR:
				tepm_instr.com_numotor.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_numotor.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_DIRECTION:
				tepm_instr.com_siderot.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_siderot.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_PWM:
				tepm_instr.com_pwm.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_pwm.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_TIME:
				tepm_instr.com_worktime.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_worktime.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_ANGLE:
				tepm_instr.com_angle.com = code;
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_angle.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_SPEED:
				tepm_instr.com_speed.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_speed.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_DELAY:
				tepm_instr.com_delay.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_delay.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_ADC:
				tepm_instr.com_adc.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_adc.value[idx] = sub_cmd[i][j++];
				}
				break;
			case PR_PROTOCOL_CODE_RUNNING:
				tepm_instr.com_running.com = code;
				uartStrc.ex_data[uartStrc.ex_counterData++] = code | (numBytes << 5);
				for (uint8_t idx = 0; idx < numBytes && idx < 10; idx++) {
					tepm_instr.com_running.value[idx] = sub_cmd[i][j++];
				}
				break;
			default:
				// Неизвестный код: пропускаем данные
				j += numBytes;
				break;
			}
		}
		module_motor_SetParam(&tepm_instr);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6) {
		uartStrc.RecvData[uartStrc.DataCount] = uartStrc.DataByte;

		/*************************************************************************************************/
		/* Прием пакета команд */
		if ((uartStrc.RecvData[uartStrc.DataCount - 1] == PR_PROTOCOL_UART_START[0])
				&& (uartStrc.RecvData[uartStrc.DataCount] == PR_PROTOCOL_UART_START[1])) {
			uartStrc.flag_Recv= true;
			uartStrc.DataCount = 0;
		}
		else if ((uartStrc.RecvData[uartStrc.DataCount - 1] == PR_PROTOCOL_UART_STOP[0])
				&& (uartStrc.RecvData[uartStrc.DataCount] == PR_PROTOCOL_UART_STOP[1])) {
			uartStrc.flag_Recv = false;

			uartStrc.RecvData[uartStrc.DataCount--] = 0;
			uartStrc.RecvData[uartStrc.DataCount] = 0;

			protocol_read_raw_uart(uartStrc.RecvData, uartStrc.DataCount);

			for (uint16_t i = 0; i < uartStrc.DataCount; i++) uartStrc.RecvData[i] = 0;


			uartStrc.DataCount = 0;
		}
		else if (uartStrc.flag_Recv) uartStrc.DataCount++;
		/*************************************************************************************************/
		/* Прием пакета данных */







		else uartStrc.DataCount++;
	}
}



		/*************************************************************************************************/
//		/*ADC_DATA*/
//		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_START_pair.first)
//				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_START_pair.second)) {
//			flag_uartResending = true;
//			UsartDataCount++;
//		}
//		else if ((UsartRecvData[UsartDataCount - 1] == PR_PROTOCOL_PACK_ADC_STOP_pair.first)
//				&& (UsartRecvData[UsartDataCount] == PR_PROTOCOL_PACK_ADC_STOP_pair.second)) {
//			CDC_Transmit_FS((uint8_t*)&UsartRecvData, UsartDataCount + 1);
//			flag_uartResending = false;
//			UsartDataCount = 0;
//		}
//		else if (flag_uartResending)
//		{
//			if (UsartDataCount >= PrHand_ADC.getPackSizeData() * 2)
//			{
//				CDC_Transmit_FS((uint8_t*)&UsartRecvData, UsartDataCount + 1);
//				UsartDataCount = 0;
//			}
//			else UsartDataCount++;
//		}
//		/*************************************************************************************************/
//		else UsartDataCount++;









