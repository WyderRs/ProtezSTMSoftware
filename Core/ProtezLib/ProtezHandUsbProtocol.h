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


/*TO PC FORMAT DATA*/
#define PR_PROTOCOL_CODE_TOPC_ADC_START		{0xAA, 0x55}
#define PR_PROTOCOL_CODE_TOPC_ADC_STOP		{0x66, 0x11}




class ProtezHandUsbProtocol {
public:
	static std::vector<std::vector<uint8_t>> subPack;
	static std::vector<std::pair<uint8_t, std::vector<uint8_t>>> command;
	static std::map<uint8_t, std::vector<uint8_t>> setCommand;
	static TIM_HandleTypeDef* timer;

	static _Bool FlagDataADC;
private:

public:
	ProtezHandUsbProtocol();
	~ProtezHandUsbProtocol();

	static void readRawPack(std::vector<uint8_t>*);
	static void selectorCommand();
	static void setCommands();

	static void subCom_Config(std::vector<uint8_t>);
	static void subCom_NumberMotor();
	static void subCom_ADC();



	static void setTimerTransmiter(TIM_HandleTypeDef*);
	static void TimerTX_Start();
	static void TimerTX_Stop();


};


#endif /*__cplusplus*/

#endif /* PROTEZLIB_PROTEZHANDUSBPROTOCOL_H_ */
