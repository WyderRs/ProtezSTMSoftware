/*
 * ProtezHandUsbProtocol.cpp
 *
 *  Created on: Jul 22, 2025
 *      Author: makar
 */

#include "ProtezHandUsbProtocol.h"

uint8_t ProtezHandUsbProtocol::command[10][64] = {0, };


ProtezHandUsbProtocol::ProtezHandUsbProtocol()
{}

ProtezHandUsbProtocol::~ProtezHandUsbProtocol()
{}

void ProtezHandUsbProtocol::readRawCommand(std::vector<uint8_t>* data)
{
	uint8_t i = 0;
	uint8_t last_x = 0;
	std::queue<uint8_t> startCom;
	std::queue<uint8_t> stopCom;

	for (const auto &x : *data) {
		if ((x == PR_PROTOCOL_START.second) && (last_x == PR_PROTOCOL_START.first))
			startCom.push(i + 1);
		if ((x == PR_PROTOCOL_STOP.second) && (last_x == PR_PROTOCOL_STOP.first))
			stopCom.push(i - 1);

		last_x = x;
		i++;
	}

	uint8_t k1 = 0;
	while (!startCom.empty()) {
		uint8_t k2 = 0;
		for(uint8_t j = startCom.front(); j < stopCom.front(); j++) {
			ProtezHandUsbProtocol::command[k1][k2] = (*data)[j];
			k2++;
		}
		startCom.pop(); stopCom.pop();
		k1++;
	}

	/*
	 * После чего придумать способ для их установки, каждый параметр должен иметь возмо
	 * жность устанавливаться отдельно */

	return;
}







