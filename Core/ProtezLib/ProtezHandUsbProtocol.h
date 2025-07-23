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

#define PR_PROTOCOL_START_PACK	std::pair<uint8_t, uint8_t>(0xEA, 0xBC)
#define PR_PROTOCOL_STOP_PACK	std::pair<uint8_t, uint8_t>(0xBC, 0xAE)

#define PR_PROTOCOL_START	std::pair<uint8_t, uint8_t>(0xDE, 0xAD)
#define PR_PROTOCOL_STOP	std::pair<uint8_t, uint8_t>(0xBE, 0xEF)




class ProtezHandUsbProtocol {
public:
	static uint8_t command[10][64];

private:

public:
	ProtezHandUsbProtocol();
	~ProtezHandUsbProtocol();

	static void readRawCommand(std::vector<uint8_t>*);

	static void subCom_Config(std::vector<uint8_t>);
	static void subCom_NumberMotor();
	static void subCom_ADC();



};


#endif /*__cplusplus*/

#endif /* PROTEZLIB_PROTEZHANDUSBPROTOCOL_H_ */
