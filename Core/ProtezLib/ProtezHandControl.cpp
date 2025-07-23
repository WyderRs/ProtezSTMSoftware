/*
 * ProtezHandControl.cpp
 *
 *  Created on: Jul 21, 2025
 *      Author: makar
 */

#include "ProtezHandControl.h"



PrHand_Motor_typedef::PrHand_Motor_typedef()
{
}
PrHand_Motor_typedef::~PrHand_Motor_typedef()
{


}
/*GLOBAL FUNCTION*/
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
void PrHand_Motor_typedef::setPWMCounter(uint32_t) {}
void PrHand_Motor_typedef::setTargetSide(PrHand_MoveState trgSide)
{
	targetSide = trgSide;
	insert_pwm();
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
}



/*STATIC FUNCTION*/
/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////*/

void PrHand_Motor_typedef::StartTIM(TIM_HandleTypeDef* tim)
{
	HAL_TIM_Base_Start_IT(tim);
}
void PrHand_Motor_typedef::StopTIM(TIM_HandleTypeDef* tim)
{
	HAL_TIM_Base_Stop_IT(tim);
}




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








