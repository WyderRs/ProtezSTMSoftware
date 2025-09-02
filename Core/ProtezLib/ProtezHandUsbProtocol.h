/*
 * ProtezHandUsbProtocol.h
 *
 *  Created on: Jul 22, 2025
 *      Author: makar
 */

#ifndef PROTEZLIB_PROTEZHANDUSBPROTOCOL_H_
#define PROTEZLIB_PROTEZHANDUSBPROTOCOL_H_


#ifdef __cplusplus

#include "stm32f4xx.h"
#include <vector>
#include "stdio.h"
#include <queue>
#include <map>

#include "usbd_cdc_if.h"
#include "ProtezHandControl.h"



#define PR_PROTOCOL_START_PACK	std::pair<uint8_t, uint8_t>(0xEA, 0xBC)
#define PR_PROTOCOL_STOP_PACK	std::pair<uint8_t, uint8_t>(0xBC, 0xAE)

#define PR_PROTOCOL_START	std::pair<uint8_t, uint8_t>(0xDE, 0xAD)
#define PR_PROTOCOL_STOP	std::pair<uint8_t, uint8_t>(0xBE, 0xEF)


#define PR_PROTOCOL_MASK_NUM_BYTES	0xE0
#define PR_PROTOCOL_MASK_CODE		0x1F

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
/*Defines code values*/
/*WORKMODE*/
#define PR_VAL_WORKMODE_MANUAL			0x01
#define PR_VAL_WORKMODE_ANGLE			0x02
#define PR_VAL_WORKMODE_STATUS			0x03
/*RUNNING*/
#define PR_VAL_RUNNING_STOP				0x00
#define PR_VAL_RUNNING_START			0x01
#define PR_VAL_RUNNING_ALL_START		0x02
#define PR_VAL_RUNNING_ALL_STOP			0x03
/*SIDEPLATE*/
#define PR_VAL_SIDEPLATE_THIS			0x00
#define PR_VAL_SIDEPLATE_OTHER			0x01


#define PR_PROTOCOL_USART_START				std::pair<uint8_t, uint8_t>(0xDE, 0xAD)
#define PR_PROTOCOL_USART_STOP				std::pair<uint8_t, uint8_t>(0xBE, 0xEF)

#define PR_PROTOCOL_PACK_ADC_START_pair 	std::pair<uint8_t, uint8_t>(0xEE, 0xDD)
#define PR_PROTOCOL_PACK_ADC_STOP_pair 		std::pair<uint8_t, uint8_t>(0xCC, 0xBB)

#define PR_PROTOCOL_PACK_SPEED_START_pair 	std::pair<uint8_t, uint8_t>(0xAE, 0xDE)
#define PR_PROTOCOL_PACK_SPEED_STOP_pair	std::pair<uint8_t, uint8_t>(0x1A, 0x4B)

struct _datactrl
{
	uint8_t out_data[10000];
	uint8_t out_sub_data[1000];

	uint16_t ptr_pack_start;
	uint16_t ptr_pack_stop;

	uint16_t ptr_config_start;
	uint16_t ptr_config_data;
	uint16_t ptr_config_stop;

	uint16_t ptr_adc_start;
	uint16_t ptr_adc_data;
	uint16_t ptr_adc_stop;

	uint16_t ptr_speed_start;
	uint16_t ptr_speed_data;
	uint16_t ptr_speed_stop;

	uint32_t count_out_data;
	uint32_t count_sub_data;
};



class ProtezHandUsbProtocol {
public:
	static UART_HandleTypeDef *ProtezUART;

	static std::vector<std::vector<uint8_t>> subPack;
	static std::vector<std::pair<uint8_t, std::vector<uint8_t>>> command;
	static std::map<uint8_t, std::vector<uint8_t>> setCommand;

	static _datactrl datactrl;

	static uint8_t PackOtherSide[500];

	static TIM_HandleTypeDef* timer;
	static uint32_t tim_tx_counter_tick;
	static uint32_t tim_tx_counter_ms;
	static uint32_t tim_tx_counter_s;


	static _Bool FlagDataADC;
	static _Bool FlagDataSPEED;

	static _Bool FlagUartControl;

private:

public:
	ProtezHandUsbProtocol();
	~ProtezHandUsbProtocol();

	static void setUARTHandle(UART_HandleTypeDef *);
	static void readRawPack(const std::vector<uint8_t>);
	static void selectorCommand();
	static void setCommands();

	static void subCom_Config(std::vector<uint8_t>);
	static void subCom_NumberMotor();
	static void subCom_ADC();



	static void setTimerTransmiter(TIM_HandleTypeDef*);
	static void TimerTX_Start();
	static void TimerTX_Stop();

	static void transmitStartPackData();
	static void transmitStopPackData();


	class UsartProtocol
	{
	public:
		UsartProtocol();
		~UsartProtocol();

		static void UsartCommand();
		static void transmitUartDataStartPack();
		static void transmitUartDataStopPack();


		static std::map<uint8_t, std::vector<uint8_t>> setUsartCommand;
	};


};


#endif /*__cplusplus*/

#endif /* PROTEZLIB_PROTEZHANDUSBPROTOCOL_H_ */
