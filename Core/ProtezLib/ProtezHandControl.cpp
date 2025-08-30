/*
 * ProtezHandControl.cpp
 *
 *  Created on: Jul 21, 2025
 *      Author: makar
 */

#include "ProtezHandControl.h"


extern PrHand_Motor_typedef Motor[6];
extern TIM_HandleTypeDef htim11;

TIM_HandleTypeDef* PrHand_Motor_typedef::GLBTimer;
uint32_t PrHand_Motor_typedef::GLBTimerCounter = 0;

uint32_t PrHand_Motor_typedef::MaxTimeInterval = 0;

PrHand_Motor_typedef::PrHand_Motor_typedef(uint8_t _id) : id(_id)
{
	id = _id;
}
PrHand_Motor_typedef::~PrHand_Motor_typedef()
{


}

uint8_t PrHand_Motor_typedef::getID()
{
	return id;
}
PrHand_MotorSetState PrHand_Motor_typedef::getState()
{
	return state;
}
uint32_t PrHand_Motor_typedef::getPWM()
{
	return PWM;
}
uint32_t PrHand_Motor_typedef::getWorkTime()
{
	return CNT_WorkTime;
}
uint32_t PrHand_Motor_typedef::getWorkDelay()
{
	return CNT_WorkDelay;
}
PrHand_MoveState PrHand_Motor_typedef::getTargetSide()
{
	return targetSide;
}
PrHand_MotorSetState PrHand_Motor_typedef::CheckWorkInterval()
{
	if (CNT_WorkTimeEnd <= PrHand_Motor_typedef::GLBTimerCounter)
		return _Ending;
	else if ((CNT_WorkTimeBegin <= PrHand_Motor_typedef::GLBTimerCounter))
		return _Working;
	else return _Launched;
}
/*GLOBAL FUNCTION*/
void PrHand_Motor_typedef::setTIMHandlerInstrCounter(uint32_t cnt)
{
	GLBTimerCounter = cnt;
}
uint32_t PrHand_Motor_typedef::getTIMHandlerInstrCounter()
{
	return GLBTimerCounter;
}
void PrHand_Motor_typedef::setTIM(TIM_HandleTypeDef* tim)
{
	TIM = tim;
	return;
}
_Bool PrHand_Motor_typedef::setChannel(std::pair<PrHand_MotCh, PrHand_MotCh> chs, prHand_Direction dir)
{
	uint32_t ch[2] = {chs.first, chs.second};
	uint8_t i = 0;

	for (const auto &c : ch)
	{
		switch (c) {
			case PR_CHANNEL_1:
				if (dir == 1) CH[i] = TIM_CHANNEL_1;
				else if (dir == 2) CH[i] = TIM_CHANNEL_1;
				CCR[i] = &(TIM->Instance->CCR1);
				i++;
				break;
			case PR_CHANNEL_2:
				if (dir == 1) CH[i] = TIM_CHANNEL_2;
				else if (dir == 2) CH[i] = TIM_CHANNEL_2;
				CCR[i] = &(TIM->Instance->CCR2);
				i++;
				break;
			case PR_CHANNEL_3:
				if (dir == 1) CH[i] = TIM_CHANNEL_3;
				else if (dir == 2) CH[i] = TIM_CHANNEL_3;
				CCR[i] = &(TIM->Instance->CCR3);
				i++;
				break;
			case PR_CHANNEL_4:
				if (dir == 1) CH[i] = TIM_CHANNEL_4;
				else if (dir == 2) CH[i] = TIM_CHANNEL_4;
				CCR[i] = &(TIM->Instance->CCR4);
				i++;
				break;
		}
	}
	GPIO_Dir = dir;
	return 0;
}
void PrHand_Motor_typedef::setPWM(uint32_t pwm)
{
	if (pwm >= PR_MAX_PWM) {
		PWM = PR_MAX_PWM;
	}
	else if (pwm <= PR_MIN_PWM) {
		PWM = PR_MIN_PWM;
	}
	else PWM = pwm;

	insert_pwm();
}
void PrHand_Motor_typedef::setTargetSide(PrHand_MoveState trgSide)
{
	targetSide = trgSide;
	insert_pwm();
}
void PrHand_Motor_typedef::setWorkTime(uint16_t time)
{
	CNT_WorkTime = time;
}
void PrHand_Motor_typedef::setWorkDelay(uint16_t dtime)
{
	CNT_WorkDelay = dtime;
}


void PrHand_Motor_typedef::Start()
{
	HAL_TIM_PWM_Start_IT(TIM, CH[0]);
	HAL_TIM_PWM_Start_IT(TIM, CH[1]);
}
void PrHand_Motor_typedef::Stop()
{
	HAL_TIM_PWM_Stop_IT(TIM, CH[0]);
	HAL_TIM_PWM_Stop_IT(TIM, CH[1]);

	/*Здесь надо добавить штуку чтобы определять как остановиться (просто сбросить или в режим hold)*/

	CNT_WorkTime = 0;
	CNT_WorkDelay= 0;
	CNT_WorkTimeBegin = 0;
	CNT_WorkTimeEnd = 0;
	targetSide = PR_MS_Stop;

	ADC_MotorState = _ADC_Motor_Disable;

	state = _Released;
}
void PrHand_Motor_typedef::setParameters()
{
	CNT_WorkTimeBegin = PrHand_Motor_typedef::GLBTimerCounter + CNT_WorkDelay;
	CNT_WorkTimeEnd = CNT_WorkTimeBegin + CNT_WorkTime;
}
void PrHand_Motor_typedef::setState(PrHand_MotorSetState st)
{
	state = st;
}
void PrHand_Motor_typedef::setADCChannel(PrHand_ADC_Channels ch)
{
	ADC_Channel = ch;
}
PrHand_ADC_Channels PrHand_Motor_typedef::getADCChannel()
{
	return ADC_Channel;
}
void PrHand_Motor_typedef::enableADC()
{
	ADC_MotorState = _ADC_Motor_Enable;
}
void PrHand_Motor_typedef::disableADC()
{
	ADC_MotorState = _ADC_Motor_Disable;
}
PrHand_MotorADCState PrHand_Motor_typedef::getEnabledADC()
{
	return ADC_MotorState;
}
/*STATIC FUNCTION*/
/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/

///*Инициализация АЦП.*/
//static void StartADC();							/*Запуск АЦП.*/
//static void StopADC();							/*Остановка АЦП.*/
//static void setStateADC(PrHand_GLB_ADCState);	/*Установка состояния АЦП.*/
//static PrHand_GLB_ADCState getStateADC();		/*Возвращает текущее состояние АЦП.*/
//
//void PrHand_Motor_typedef::StartADC()
//{
//
//	GLB_ADC_state = _ADC_Working;
//}
//void PrHand_Motor_typedef::StopADC()
//{
//
//	GLB_ADC_state = _ADC_Released;
//}
//void PrHand_Motor_typedef::setStateADC(PrHand_GLB_ADCState st)
//{
//	GLB_ADC_state = st;
//}
//PrHand_GLB_ADCState PrHand_Motor_typedef::getStateADC()
//{
//	for (auto &motor : Motor) {
//		if (motor.ADC_MotorState == _ADC_Motor_Enable)  return GLB_ADC_state = _ADC_NoConfigured;
//	}
//	return GLB_ADC_state = _ADC_Disable;
//}
void PrHand_Motor_typedef::SetTIMHandlerInstr(TIM_HandleTypeDef* tim)
{
	GLBTimer = tim;
}
void PrHand_Motor_typedef::StartTIMHandlerInstr()
{
	HAL_TIM_Base_Start_IT(GLBTimer);
}
void PrHand_Motor_typedef::StopTIMHandlerInstr()
{
	HAL_TIM_Base_Stop_IT(GLBTimer);
}
//static void StartADC();
//static void StopADC();



/*PRIVATE FUNCTION*/
/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/
void PrHand_Motor_typedef::insert_pwm()
{
	if (targetSide == PR_MS_Stop)
	{
		*CCR[0] = PR_MIN_PWM;
		*CCR[1] = PR_MIN_PWM;
	}
	else if (targetSide == PR_MS_Hold)
	{
		*CCR[0] = PR_MAX_PWM;
		*CCR[1] = PR_MAX_PWM;
	}
	else if (targetSide == PR_MS_Left)
	{
		*CCR[0] = PWM;
		*CCR[1] = PR_MIN_PWM;
	}
	else if (targetSide == PR_MS_Right)
	{
		*CCR[0] = PR_MIN_PWM;
		*CCR[1] = PWM;
	}
}








