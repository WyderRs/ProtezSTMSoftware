/*
 * ProtezHandUsbProtocol.cpp
 *
 *  Created on: Jul 22, 2025
 *      Author: makar
 */

#include "ProtezHandUsbProtocol.h"
#include "ProtezHandADC.h"


std::vector<std::vector<uint8_t>> ProtezHandUsbProtocol::subPack;
std::vector<std::pair<uint8_t, std::vector<uint8_t>>> ProtezHandUsbProtocol::command;
std::map<uint8_t, std::vector<uint8_t>> ProtezHandUsbProtocol::setCommand;

TIM_HandleTypeDef* ProtezHandUsbProtocol::timer;
_Bool ProtezHandUsbProtocol::FlagDataADC;

/*USER DEFINES*/
extern PrHand_Motor_typedef Motor[6];

/*END USER DEFINES*/

ProtezHandUsbProtocol::ProtezHandUsbProtocol()
{}

ProtezHandUsbProtocol::~ProtezHandUsbProtocol()
{}

void ProtezHandUsbProtocol::readRawPack(std::vector<uint8_t>* data)
{
	uint8_t i = 0;
	uint8_t last_x = 0;
	std::queue<uint8_t> startCom;
	std::queue<uint8_t> stopCom;

	if (data->empty()) return;

	for (const auto &x : *data) {
		if ((x == PR_PROTOCOL_START.second) && (last_x == PR_PROTOCOL_START.first))
			startCom.push(i + 1);
		if ((x == PR_PROTOCOL_STOP.second) && (last_x == PR_PROTOCOL_STOP.first))
			stopCom.push(i - 1);
		last_x = x;
		i++;
	}
	while (!startCom.empty()) {
		uint8_t k1 = 0;
	    std::vector<uint8_t> innerVec;
		for(uint8_t j = startCom.front(); j < stopCom.front(); j++) {
			innerVec.push_back((*data)[j]);
			k1++;
		}
		ProtezHandUsbProtocol::subPack.push_back(innerVec);
		startCom.pop(); stopCom.pop();
	}

	ProtezHandUsbProtocol::selectorCommand();
	ProtezHandUsbProtocol::setCommands();

	return;
}
void ProtezHandUsbProtocol::selectorCommand()
{
    ProtezHandUsbProtocol::command.clear();

    for (const auto &x : ProtezHandUsbProtocol::subPack) {
        if (x.empty()) continue;

        size_t i = 0;
        while (i < x.size()) {
            uint8_t header = x[i];
            uint8_t code = header & PR_PROTOCOL_MASK_CODE;
            uint8_t numBytes = (header & PR_PROTOCOL_MASK_NUM_BYTES) >> 5;

            if (i + 1 + numBytes > x.size()) {
                break;
            }

            std::vector<uint8_t> data;
            for (size_t j = 0; j < numBytes; ++j) {
                data.push_back(x[i + 1 + j]);
            }

            switch (code) {
                case PR_PROTOCOL_CODE_WORKMODE:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_CONFIG:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_SIDEPLATE:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_NUMMOTOR:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_DIRECTION:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_PWM:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_TIME:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_DELAY:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_ANGLE:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_SPEED:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_FEEDBACK:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_ADC:
                	ProtezHandUsbProtocol::command.push_back({code, data});
                	ProtezHandUsbProtocol::setCommand[code] = data;
                	break;
                case PR_PROTOCOL_CODE_RUNNING:
                    ProtezHandUsbProtocol::command.push_back({code, data});
                    ProtezHandUsbProtocol::setCommand[code] = data;
                    break;
                default:
                    // Неизвестный код — можно залогировать или пропустить
                    break;
            }
            i += 1 + numBytes;
        }
    }

}

void ProtezHandUsbProtocol::setCommands()
{
	// если другая сторона платы то требуется отправить данный пакет по uart
//	PR_PROTOCOL_CODE_SIDEPLATE;

	/*Получение стороны платы*/
	int8_t sidePlate = -1;
	auto itsidePlate = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_SIDEPLATE);
	if (itsidePlate != ProtezHandUsbProtocol::setCommand.end()) {
		sidePlate = itsidePlate->second[0];
	}

	/*Тип управления*/
	int8_t wmMotor = -1;
	auto wm = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_WORKMODE);
	if (wm != ProtezHandUsbProtocol::setCommand.end()) {
		wmMotor = wm->second[0];
	}

	/*Получение номера двигателя*/
	int8_t numMotor = -1;
	auto itNumMotor = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_NUMMOTOR);
	if (itNumMotor != ProtezHandUsbProtocol::setCommand.end()) {
		numMotor = itNumMotor->second[0];
	}
	else { /*Если байт настройки номера двигателя пустой (т.е. команды вообще нет)*/
		char buf[80] = "ERROR: There is no package with the engine number.";
		CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
		return;
	}

	/*Получение направления вращения*/
	int8_t motDir = -1;
	auto itMotDir = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_DIRECTION);
	if (itMotDir != ProtezHandUsbProtocol::setCommand.end()) {
		motDir = itMotDir->second[0];
	}
	/*Получение PWM*/
	int16_t motPWM = -1;
	auto itMotPWM = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_PWM);
	if (itMotPWM != ProtezHandUsbProtocol::setCommand.end()) {
		motPWM = itMotPWM->second[0] * 10;
	}
	/*Получение Time*/
	int16_t motTime = -1;
	auto itMotTime = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_TIME);
	if (itMotTime != ProtezHandUsbProtocol::setCommand.end()) {
		uint8_t stp = 0;
		motTime = 0;
		for (auto t : itMotTime->second) {
			motTime += (t << stp);
			stp += 8;
		}
	}
	/*Получение Delay*/
	int16_t motDelay = -1;
	auto itMotDelay = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_DELAY);
	if (itMotDelay != ProtezHandUsbProtocol::setCommand.end()) {
		uint8_t stp = 0;
		motDelay = 0;
		for (auto t : itMotDelay->second) {
			motDelay += (t << stp);
			stp += 8;
		}
	}
	/*Получение Angle*/
	int16_t motAngle = -1;
	auto itMotAngle = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_ANGLE);
	if (itMotAngle != ProtezHandUsbProtocol::setCommand.end()) {
		uint8_t stp = 0;
		motAngle = 0;
		for (auto t : itMotAngle->second) {
			motAngle += (t << stp);
			stp += 8;
		}
	}
	/*Получение Speed*/
	int16_t motSpeed = -1;
	auto itMotSpeed = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_SPEED);
	if (itMotSpeed != ProtezHandUsbProtocol::setCommand.end()) {
		uint8_t stp = 0;
		motSpeed = 0;
		for (auto t : itMotSpeed->second) {
			motSpeed += (t << stp);
			stp += 8;
		}
	}
	/*Получение Feedback*/
	int16_t motFeedback = -1;
	auto itMotFeedback = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_FEEDBACK);
	if (itMotFeedback != ProtezHandUsbProtocol::setCommand.end()) {
		// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
		motFeedback = itMotFeedback->second[0];
	}
	/*Получение ADC*/
	int16_t motADC = -1;
	auto itMotADC = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_ADC);
	if (itMotADC != ProtezHandUsbProtocol::setCommand.end()) {
		// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
		motADC = itMotADC->second[0];
	}
	/*Получение Running*/
	int16_t motRUN = -1;
	auto itMotRUN = ProtezHandUsbProtocol::setCommand.find(PR_PROTOCOL_CODE_RUNNING);
	if (itMotRUN != ProtezHandUsbProtocol::setCommand.end()) {
		// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
		motRUN = itMotRUN->second[0];
	}

	if (wmMotor == PR_VAL_WORKMODE_MANUAL)
	{
//		if (numMotor != -1) Motor[numMotor].getID();
		if (motDir != -1) Motor[numMotor].setTargetSide((PrHand_MoveState)motDir);
		if (motPWM != -1)
		{
			Motor[numMotor].setPWM(motPWM);

			if ((Motor[numMotor].getPWM() != 0) && (Motor[numMotor].getWorkTime() != 0))
				Motor[numMotor].setState(_Configured);
		}
		if (motTime != -1)
		{
			Motor[numMotor].setWorkTime(motTime);

			if ((Motor[numMotor].getPWM() != 0) && (Motor[numMotor].getWorkTime() != 0))
				Motor[numMotor].setState(_Configured);
		}
		if (motDelay != -1) Motor[numMotor].setWorkDelay(motDelay);
		if (motADC != -1)
		{
			if (motADC == 1)
			{
				Motor[numMotor].enableADC();
			}
			else Motor[numMotor].disableADC();
		}
		if (motRUN != -1)
		{
			/*1. Стоп двигатель
			 *2. Старт двигатель
			 *3. Старт все двигатели
			 *4. Стоп все двигатели */
			if (motRUN & PR_VAL_RUNNING_STOP) Motor[numMotor].Stop();
			else if (motRUN & PR_VAL_RUNNING_START) {
				if (Motor[numMotor].getState() == _Configured) {
					Motor[numMotor].setParameters();
					Motor[numMotor].setState(_Launched);
				}
			}
			else if (motRUN & PR_VAL_RUNNING_ALL_START) {
				for (auto motor : Motor)
					if (motor.getState() == _Configured) {
						motor.setParameters();
						motor.setState(_Launched);
					}
			}
			else if (motRUN & PR_VAL_RUNNING_ALL_STOP)
				for (auto motor : Motor)
					if ((motor.getState() == _Working) || (motor.getState() == _Launched)) motor.Stop();
		}
	}
	else if (wmMotor == PR_VAL_WORKMODE_ANGLE)
	{
//		if (numMotor != -1) Motor[numMotor].getID();
//		if (motDir != -1) Motor[numMotor].setTargetSide((PrHand_MoveState)motDir);
//		if (motTime != -1) Motor[numMotor].setTime(motTime);
//		if (motDelay != -1) Motor[numMotor].setDelay(motDelay);
//		if (motAngle != -1)	Motor[numMotor].setAngle(motAngle);
//		if (motSpeed != -1)	Motor[numMotor].setSpeed(motSpeed);
//		if (motFeedback != -1) Motor[numMotor].setFeedback(motFeedback);
//		if (motADC != -1) Motor[numMotor].setADC(motADC);
//		if (motRUN != -1) Motor[numMotor].setRUN(motRUN);
	}
	else if (wmMotor == PR_VAL_WORKMODE_STATUS)	// ТУТ ДОЛЖНЫ БЫТЬ ГЕТТЕРЫ
	{
//		if (numMotor != -1) Motor[numMotor].getID();
//		if (motDir != -1) Motor[numMotor].setTargetSide((PrHand_MoveState)motDir);
//		if (motPWM != -1) Motor[numMotor].setPWM(motPWM);
//		if (motTime != -1) Motor[numMotor].setTime(motTime);
//		if (motDelay != -1) Motor[numMotor].setDelay(motDelay);
//		if (motAngle != -1)	Motor[numMotor].setAngle(motAngle);
//		if (motSpeed != -1)	Motor[numMotor].setSpeed(motSpeed);
//		if (motFeedback != -1) Motor[numMotor].setFeedback(motFeedback);
//		if (motADC != -1) Motor[numMotor].setADC(motADC);
//		if (motRUN != -1) Motor[numMotor].setRUN(motRUN);
	}

//	Motor[numMotor]


}

void ProtezHandUsbProtocol::setTimerTransmiter(TIM_HandleTypeDef* tim)
{
	timer = tim;
}
void ProtezHandUsbProtocol::TimerTX_Start()
{
	HAL_TIM_Base_Start_IT(timer);
}
void ProtezHandUsbProtocol::TimerTX_Stop()
{
	HAL_TIM_Base_Stop_IT(timer);
}















