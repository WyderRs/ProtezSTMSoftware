/*
 * ProtezHandUsbProtocol.cpp
 *
 *  Created on: Jul 22, 2025
 *      Author: makar
 */

#include "ProtezHandUsbProtocol.h"
#include "ProtezHandADC.h"



UART_HandleTypeDef *ProtezHandUsbProtocol::ProtezUART;

/*TO PC FORMAT DATA*/


uint8_t PR_PROTOCOL_PACK_DATA_START[2] = {0xEE, 0xDD};
uint8_t PR_PROTOCOL_PACK_DATA_STOP[2] = {0xCC, 0xBB};


std::vector<std::vector<uint8_t>> ProtezHandUsbProtocol::subPack;
std::vector<std::pair<uint8_t, std::vector<uint8_t>>> ProtezHandUsbProtocol::command;
std::map<uint8_t, std::vector<uint8_t>> ProtezHandUsbProtocol::setCommand;
std::map<uint8_t, std::vector<uint8_t>> ProtezHandUsbProtocol::UsartProtocol::setUsartCommand;


TIM_HandleTypeDef* ProtezHandUsbProtocol::timer;
uint32_t ProtezHandUsbProtocol::tim_tx_counter_tick = 0;
uint32_t ProtezHandUsbProtocol::tim_tx_counter_ms = 0;
uint32_t ProtezHandUsbProtocol::tim_tx_counter_s = 0;

_Bool ProtezHandUsbProtocol::FlagDataADC;
_Bool ProtezHandUsbProtocol::FlagDataSPEED;
_Bool ProtezHandUsbProtocol::FlagUartControl;

uint8_t ProtezHandUsbProtocol::PackOtherSide[500];

_datactrl ProtezHandUsbProtocol::datactrl;


//std::queue<std::pair<uint8_t&, uint32_t>> ProtezHandUsbProtocol::Queue_SendData;

/*USER DEFINES*/
extern PrHand_Motor_typedef Motor[6];

/*END USER DEFINES*/

ProtezHandUsbProtocol::ProtezHandUsbProtocol()
{}

ProtezHandUsbProtocol::~ProtezHandUsbProtocol()
{}
void ProtezHandUsbProtocol::setUARTHandle(UART_HandleTypeDef *uart)
{
	ProtezUART = uart;
}

void ProtezHandUsbProtocol::readRawPack(std::vector<uint8_t> data)
{
	uint8_t i = 0;
	uint8_t last_x = 0;
	std::queue<uint8_t> startCom;
	std::queue<uint8_t> stopCom;


//	uint8_t mymass2[100] = {0, };
//	uint16_t r = 0;
//	for (auto x : data)
//	{
//		mymass2[r] = x;
//		r++;
//	}


	if (data.empty()) return;

	for (const auto &x : data) {
		if ((x == PR_PROTOCOL_START.second) && (last_x == PR_PROTOCOL_START.first))
			startCom.push(i + 1);

		if ((x == PR_PROTOCOL_STOP.second) && (last_x == PR_PROTOCOL_STOP.first))
			stopCom.push(i - 1);

		last_x = x;
		i++;
	}
	while (!startCom.empty()) {
		uint16_t k1 = 0;
	    std::vector<uint8_t> innerVec;
		for(uint8_t j = startCom.front(); j < stopCom.front(); j++) {
			innerVec.push_back((data)[j]);
			k1++;
		}

//		uint8_t mymass[100] = {0, };
//		uint16_t i = 0;
//		for (auto x : innerVec)
//		{
//			mymass[i] = x;
//			i++;
//		}

		ProtezHandUsbProtocol::subPack.push_back(innerVec);
		startCom.pop(); stopCom.pop();
	}

	ProtezHandUsbProtocol::selectorCommand();
	return;
}
void ProtezHandUsbProtocol::selectorCommand()
{
    ProtezHandUsbProtocol::command.clear();
    ProtezHandUsbProtocol::setCommand.clear();

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

    	ProtezHandUsbProtocol::setCommands();
    }

    /*Определяем максимальное время работы*/
    PrHand_Motor_typedef::MaxTimeInterval = 0;
    PrHand_Motor_typedef::ConfiguredMotor = 0;
   	uint32_t temp_countEnabled = 0;
    for (auto &mot : Motor)
    {
    	if (mot.getWorkTime() * 10 > PrHand_Motor_typedef::MaxTimeInterval)
    	{
    		PrHand_Motor_typedef::MaxTimeInterval = mot.getWorkTime() * 10;
    	}
    	if ((mot.getState() == _Configured) || (mot.getState() == _Launched))
    	{
    		PrHand_Motor_typedef::ConfiguredMotor++;
    	}
    }


    /*Устанавливаем стартовые позиции частей данных*/
    ProtezHandUsbProtocol::datactrl.ptr_pack_start = 0; 	/*Расположение стартовых байтов пакета*/
    ProtezHandUsbProtocol::datactrl.ptr_pack_stop = 0;		/*Расположение стоповых байтов пакета*/

    ProtezHandUsbProtocol::datactrl.ptr_config_start = 0;	/*Расположение стартовых байтов данных конфига*/
    ProtezHandUsbProtocol::datactrl.ptr_config_data = 0;	/*Расположение данных конфига*/
    ProtezHandUsbProtocol::datactrl.ptr_config_stop = 0;	/*Расположение стопоовых байтов данных конфига*/

    ProtezHandUsbProtocol::datactrl.ptr_adc_start = 0;		/*Расположение стартовых байтов данных ацп*/
    ProtezHandUsbProtocol::datactrl.ptr_adc_data = 0;		/*Расположение данных ацп*/
    ProtezHandUsbProtocol::datactrl.ptr_adc_stop = 0;		/*Расположение стопоовых байтов данных ацп*/
    ProtezHandUsbProtocol::datactrl.ptr_speed_start= 0;		/*Расположение стартовых байтов данных энкодера*/
    ProtezHandUsbProtocol::datactrl.ptr_speed_data = 0;		/*Расположение данных энкодера*/
    ProtezHandUsbProtocol::datactrl.ptr_speed_stop = 0;		/*Расположение стопоовых байтов данных энкодера*/

    ProtezHandUsbProtocol::datactrl.count_out_data = 0;		/*Устанавливаем общий размер посылки*/
    ProtezHandUsbProtocol::datactrl.count_sub_data = PrHand_Motor_typedef::ConfiguredMotor * 40;		/*Устанавливаем размер промежуточных данных*/

    ProtezHandUsbProtocol::subPack.clear();
}

void ProtezHandUsbProtocol::setCommands()
{
	// если другая сторона платы то требуется отправить данный пакет по uart
//	PR_PROTOCOL_CODE_SIDEPLATE;

	/*Получение стороны платы*/
	int8_t sidePlate = -1;
	auto itsidePlate = setCommand.find(PR_PROTOCOL_CODE_SIDEPLATE);
	if (itsidePlate != setCommand.end()) {
		sidePlate = itsidePlate->second[0];
	}
//	if (sidePlate == PR_VAL_SIDEPLATE_THIS)
//	{
		/*Тип управления*/
		int8_t wmMotor = -1;
		auto wm = setCommand.find(PR_PROTOCOL_CODE_WORKMODE);
		if (wm != setCommand.end()) {
			wmMotor = wm->second[0];
		}

		/*Получение номера двигателя*/
		int8_t numMotor = -1;
		auto itNumMotor = setCommand.find(PR_PROTOCOL_CODE_NUMMOTOR);
		if (itNumMotor != setCommand.end()) {
			numMotor = itNumMotor->second[0];
		}
		else { /*Если байт настройки номера двигателя пустой (т.е. команды вообще нет)*/
			char buf[80] = "ERROR: There is no package with the engine number.";
			CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
			return;
		}

		/*Получение направления вращения*/
		int8_t motDir = -1;
		auto itMotDir = setCommand.find(PR_PROTOCOL_CODE_DIRECTION);
		if (itMotDir != setCommand.end()) {
			motDir = itMotDir->second[0];
		}
		/*Получение PWM*/
		int16_t motPWM = -1;
		auto itMotPWM = setCommand.find(PR_PROTOCOL_CODE_PWM);
		if (itMotPWM != setCommand.end()) {
			motPWM = itMotPWM->second[0] * 10;
		}
		/*Получение Time*/
		int16_t motTime = -1;
		auto itMotTime = setCommand.find(PR_PROTOCOL_CODE_TIME);
		if (itMotTime != setCommand.end()) {
			uint8_t stp = 0;
			motTime = 0;
			for (auto t : itMotTime->second) {
				motTime += (t << stp);
				stp += 8;
			}
		}
		/*Получение Delay*/
		int16_t motDelay = -1;
		auto itMotDelay = setCommand.find(PR_PROTOCOL_CODE_DELAY);
		if (itMotDelay != setCommand.end()) {
			uint8_t stp = 0;
			motDelay = 0;
			for (auto t : itMotDelay->second) {
				motDelay += (t << stp);
				stp += 8;
			}
		}
		/*Получение Angle*/
		int16_t motAngle = -1;
		auto itMotAngle = setCommand.find(PR_PROTOCOL_CODE_ANGLE);
		if (itMotAngle != setCommand.end()) {
			uint8_t stp = 0;
			motAngle = 0;
			for (auto t : itMotAngle->second) {
				motAngle += (t << stp);
				stp += 8;
			}
		}
		/*Получение Speed*/
		int16_t motSpeed = -1;
		auto itMotSpeed = setCommand.find(PR_PROTOCOL_CODE_SPEED);
		if (itMotSpeed != setCommand.end()) {
			uint8_t stp = 0;
			motSpeed = 0;
			for (auto t : itMotSpeed->second) {
				motSpeed += (t << stp);
				stp += 8;
			}
		}
		/*Получение Feedback*/
		int16_t motFeedback = -1;
		auto itMotFeedback = setCommand.find(PR_PROTOCOL_CODE_FEEDBACK);
		if (itMotFeedback != setCommand.end()) {
			// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
			motFeedback = itMotFeedback->second[0];
		}
		/*Получение ADC*/
		int16_t motADC = -1;
		auto itMotADC = setCommand.find(PR_PROTOCOL_CODE_ADC);
		if (itMotADC != setCommand.end()) {
			// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
			motADC = itMotADC->second[0];
		}
		/*Получение Running*/
		int16_t motRUN = -1;
		auto itMotRUN = setCommand.find(PR_PROTOCOL_CODE_RUNNING);
		if (itMotRUN != setCommand.end()) {
			// ЗДЕСЬ НАДО ДОБАВИТЬ ЦИКЛ
			motRUN = itMotRUN->second[0];
		}

		if (wmMotor == PR_VAL_WORKMODE_MANUAL)
		{
	//		if (numMotor != -1) Motor[numMotor].getID();


			if (motDir != -1)
			{
				Motor[numMotor].setTargetSide((PrHand_MoveState)motDir);
			}
			if (Motor[numMotor].getState() != _Working)
			{
				if (motPWM != -1)
				{
					if ((motDir == PR_MS_Right) && (!ProtezHandUsbProtocol::FlagUartControl))
					{
						Motor[numMotor].setPWM(8 * 10);
					}
					else Motor[numMotor].setPWM(motPWM);

					if ((Motor[numMotor].getPWM() != 0) && (Motor[numMotor].getWorkTime() != 0))
					{
						Motor[numMotor].setState(_Configured);
					}
				}
				if (motTime != -1)
				{
					Motor[numMotor].setWorkTime(motTime);

					if ((Motor[numMotor].getPWM() != 0) && (Motor[numMotor].getWorkTime() != 0))
					{
						Motor[numMotor].setState(_Configured);
					}
				}
				if (motDelay != -1)
					Motor[numMotor].setWorkDelay(motDelay);
			}
			if (motADC != -1)
			{
				if (motADC == 1)
				{
					if (motDir != PR_MS_Right)
					{
						Motor[numMotor].enableADC();
					}
					else if (ProtezHandUsbProtocol::FlagUartControl)
					{
						Motor[numMotor].enableADC();
					}
				}
				else Motor[numMotor].disableADC();
			}
			if (motRUN != -1)
			{
				/*1. Стоп двигатель
				 *2. Старт двигатель
				 *3. Старт все двигатели
				 *4. Стоп все двигатели */




//				for (uint8_t i = 0; i < 6; i++) sideFinger[i] = PR_MS_Stop;
//				for (auto &x : Motor)
//				{
//					PrHand_MoveState side = x.getTargetSide();
//					sideFinger[x.getID()] = side;
//					if (ProtezHandUsbProtocol::FlagUartControl)
//					{
//						numThisMotor++;
//					}
//					else if (side == PR_MS_Left)
//					{
//						numThisMotor++;
//					}
//
//				}

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

//}
	if (sidePlate == PR_VAL_SIDEPLATE_OTHER)
	{
		uint16_t cc_d = 0;
		for (auto part : setCommand)
		{
			std::vector<uint8_t> data(0);

			switch (part.first) {
			case PR_PROTOCOL_CODE_WORKMODE:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_CONFIG:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_SIDEPLATE:
				data.push_back(0); data.push_back(0);
				ProtezHandUsbProtocol::setCommand[part.first] = data;
				data.clear();
				break;
			case PR_PROTOCOL_CODE_NUMMOTOR:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_DIRECTION:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_PWM:
				if (motDir == PR_MS_Left)
				{
					data.push_back(8); data.push_back(0);
					ProtezHandUsbProtocol::setCommand[part.first] = data;
				}
				else
				{
					ProtezHandUsbProtocol::setCommand[part.first];
				}

				data.clear();
				break;
			case PR_PROTOCOL_CODE_TIME:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_DELAY:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_ANGLE:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_SPEED:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_FEEDBACK:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			case PR_PROTOCOL_CODE_ADC:

				if (motDir == PR_MS_Left)
				{
					data.push_back(0);
					ProtezHandUsbProtocol::setCommand[part.first] = data;
				}
				else ProtezHandUsbProtocol::setCommand[part.first];

				break;
			case PR_PROTOCOL_CODE_RUNNING:
				ProtezHandUsbProtocol::setCommand[part.first];
				break;
			}

		}

		PackOtherSide[cc_d] = PR_PROTOCOL_USART_START.first;
		cc_d++;
		PackOtherSide[cc_d] = PR_PROTOCOL_USART_START.second;
		cc_d++;
		for (auto &cmd : setCommand) {
			PackOtherSide[cc_d] = cmd.first | (cmd.second.size() << 5);
			cc_d++;
			for (auto &subcmd : cmd.second) {
				PackOtherSide[cc_d] = subcmd;
				cc_d++;
			}
		}
		PackOtherSide[cc_d] = PR_PROTOCOL_USART_STOP.first;
		cc_d++;
		PackOtherSide[cc_d] = PR_PROTOCOL_USART_STOP.second;
		cc_d++;

		HAL_UART_Transmit_IT(ProtezUART, PackOtherSide, cc_d);
	}

	ProtezHandUsbProtocol::setCommand.clear();
}

void ProtezHandUsbProtocol::setTimerTransmiter(TIM_HandleTypeDef* tim)
{
	timer = tim;
}
void ProtezHandUsbProtocol::TimerTX_Start()
{
	tim_tx_counter_tick = 0;
	tim_tx_counter_ms = 0;
	tim_tx_counter_s = 0;
	HAL_TIM_Base_Start_IT(timer);
}
void ProtezHandUsbProtocol::TimerTX_Stop()
{
	HAL_TIM_Base_Stop_IT(timer);
}
void ProtezHandUsbProtocol::transmitStartPackData()
{
	CDC_Transmit_FS(PR_PROTOCOL_PACK_DATA_START, 2);
}
void ProtezHandUsbProtocol::transmitStopPackData()
{
	CDC_Transmit_FS(PR_PROTOCOL_PACK_DATA_STOP, 2);
}

/*----------*/
/*USART PROTOCOL*/
/*----------*/
ProtezHandUsbProtocol::UsartProtocol::UsartProtocol() {}
ProtezHandUsbProtocol::UsartProtocol::~UsartProtocol() {}

void ProtezHandUsbProtocol::UsartProtocol::UsartCommand()
{
    ProtezHandUsbProtocol::setCommands();
    ProtezHandUsbProtocol::UsartProtocol::setUsartCommand.clear();
}
void ProtezHandUsbProtocol::UsartProtocol::transmitUartDataStartPack()
{
	HAL_UART_Transmit_IT(ProtezUART, PR_PROTOCOL_PACK_DATA_START, 2);
}
void ProtezHandUsbProtocol::UsartProtocol::transmitUartDataStopPack()
{
	HAL_UART_Transmit_IT(ProtezUART, PR_PROTOCOL_PACK_DATA_STOP, 2);
}



