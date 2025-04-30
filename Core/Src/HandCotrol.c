/*
 * CyberMed_HandCotrol.c
 *
 *  Created on: Mar 11, 2025
 *      Author: Roman
 */


#include "HandCotrol.h"
#include <stdbool.h>

/*Global variables*/
extern volatile uint32_t GLB_Time[3];	// Counter time: [0] - 1s, [1] - 0.1s, [2] - 0.01s
uint32_t TEST_cntTim2 = 0;				// Test variables (counter TIM2)
uint8_t ADC_Data[500] = {0, };			// Data received from ADC
//uint8_t ADC_DataSent[5000] = {0, };			// ALL Data of sent
uint32_t ADC_Channels[6] = {0, };		// Number ADC channels
uint32_t drts = 0; 						// Number data ready to send
uint32_t dstc = 0; 						// Number data sent to COM
uint32_t num_pack = 20; 				// Number data to send to 1 tick
MotorDefinition Motor[6];				// Structure of Motors
PRGlbDef ProtezGlobalConf;				// Global definitions
EncoderSens Encoder[6];					// Encoder sensors parameters
RCV_Flags FlagsRecvInst;				// Flags of received instruction
uint32_t NowCountPointADC;				// Count point of ADC data
uint32_t glb_dstc;						// ALL Transmited data			DELETE
uint32_t Target_dtsc[6];				// Target pack for send to comport
uint8_t GLB_TypeCtrl;					// Global type control
FL2_TypeCtrlMove TCM;					// Type control moving motor

uint8_t UsartDataByte[30];					// Usart byte
uint8_t UsartData[40];					// Usart Data
uint32_t UsartDataCnt;					// Usart Data count
_Bool UART_CommandRecieved = false;

_Bool FlagDMA_START = false;			// Start DMA reading
_Bool DeviceIsConnected = false;		// USB Device is connected
_Bool ThisDeviceOnUsartCtrl = false;	// This device on usart control
//*****************//
// end-to-end (ETE MODE)
_Bool ETEMode_Enable = false;

//*****************//
/**************************************************************************************/

/**************************************************************************************/
/*PID Regulator*/
double Coef_P = 0.05;
double Coef_I = 0.05;
double Coef_T = 100.0;
uint16_t ContRegulatorValue;
uint32_t TEST_GLB_TIM10_CNT1;
/**************************************************************************************/
extern uint32_t EncTime[6];
extern uint32_t EncTimeNow[6];
extern uint32_t EncTimeOld[6];


extern uint32_t EncCnt[6];
extern uint32_t EncCntNow[6];
extern uint32_t EncCntOld[6];
extern uint32_t d_EncCnt[6];

extern double RegVal[1000];
extern uint32_t d_EncTime[6][500];


/*Extern definition*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim11;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart6;




/*
	1. Реализация памяти. Если выполнение инструкции остановили, то можно вернуться и довыполнить ее.

*/

MotorDefinition Motor_Settings(TIM_HandleTypeDef *htim, PairChannelOut prch,_Bool invert)
{
	MotorDefinition motor = {0, };
	/*Set TIM*/
	motor.md_htim = htim;
	/*Set pair channels*/
	if(!invert)
	{
		if (prch == PAIRCHANNEL_1)
		{
			motor.md_chl = CHANNEL_1;
			motor.md_chr = CHANNEL_2;
		}
		else if (prch == PAIRCHANNEL_2)
		{
			motor.md_chl = CHANNEL_3;
			motor.md_chr = CHANNEL_4;
		}
	}
	else if (invert)
	{
		if (prch == PAIRCHANNEL_1)
		{
			motor.md_chl = CHANNEL_2;
			motor.md_chr = CHANNEL_1;
		}
		else if (prch == PAIRCHANNEL_2)
		{
			motor.md_chl = CHANNEL_4;
			motor.md_chr = CHANNEL_3;
		}
	}
	/*Set side rotation*/
	motor.md_rotsd = FREE;
	/*Set default CRRN value*/
	motor.md_chl_value = 0;
	motor.md_chr_value = 0;
	/*Set no work state*/
	motor.md_st = RELEASED;
	motor.md_prch = prch;
	return motor;
}

void FL_1_Motor_SetDuty(MotorDefinition *motor, uint32_t duty_l, uint32_t duty_r)
{
	motor->md_chl_value = duty_l;
	motor->md_chr_value = duty_r;
	if(motor->md_prch == PAIRCHANNEL_1)
	{
		motor->md_htim->Instance->CCR1 = duty_l;
		motor->md_htim->Instance->CCR2 = duty_r;
	}
	else if(motor->md_prch == PAIRCHANNEL_2)
	{
		motor->md_htim->Instance->CCR3 = duty_l;
		motor->md_htim->Instance->CCR4 = duty_r;
	}
}
void FL_2_Motor_SetDuty(MotorDefinition *motor, uint32_t duty_l, uint32_t duty_r)
{
	motor->md_chl_value = duty_l;
	motor->md_chr_value = duty_r;
	if(motor->md_prch == PAIRCHANNEL_1)
	{
		motor->md_htim->Instance->CCR1 = duty_l;
		motor->md_htim->Instance->CCR2 = duty_r;
	}
	else if(motor->md_prch == PAIRCHANNEL_2)
	{
		motor->md_htim->Instance->CCR3 = duty_l;
		motor->md_htim->Instance->CCR4 = duty_r;
	}
}
void FL_1_Motor_ContinuousDuty(MotorDefinition *motor)
{
	if(motor->md_prch == PAIRCHANNEL_1)
	{
		motor->md_htim->Instance->CCR1 = motor->md_chl_value;
		motor->md_htim->Instance->CCR2 = motor->md_chr_value;
	}
	else if(motor->md_prch == PAIRCHANNEL_2)
	{
		motor->md_htim->Instance->CCR3 = motor->md_chl_value;
		motor->md_htim->Instance->CCR4 = motor->md_chr_value;
	}
}
void FL_2_Motor_ContinuousDuty(MotorDefinition *motor)
{
//	if(EncTime[0] == 0) EncTime[0] = 1;
//	if(d_EncTime[0][Motor[0].md_encod_sn.cnt] == 0) ContRegulatorValue = 10; // d_EncCnt[0]
//	else
//	{
//		cccc[Motor[0].md_encod_sn.cnt] = -Coef_P * ((60.0 / 1.0) - (10000.0 / (1.0 * d_EncTime[0][Motor[0].md_encod_sn.cnt])));
//		ContRegulatorValue = ContRegulatorValue + (int16_t)(cccc[Motor[0].md_encod_sn.cnt]);
//	}
//	SpeedAngleMas[Motor[0].md_encod_sn.cnt] = ContRegulatorValue;

	if(ContRegulatorValue > 0)
	{
		motor->md_chr_value = ContRegulatorValue;
		motor->md_chl_value = 0;
	}
	if(motor->md_prch == PAIRCHANNEL_1)
	{
		motor->md_htim->Instance->CCR1 = motor->md_chl_value;
		motor->md_htim->Instance->CCR2 = motor->md_chr_value;
	}
	else if(motor->md_prch == PAIRCHANNEL_2)
	{
		motor->md_htim->Instance->CCR3 = motor->md_chl_value;
		motor->md_htim->Instance->CCR4 = motor->md_chr_value;
	}
}

void FL_1_Motor_SetTimeSettings(MotorDefinition *motor)
{
	motor->md_startWorkTime = motor->md_delayTime + GLB_Time[2];
	motor->md_stopWorkTime = motor->md_startWorkTime + motor->md_workTime;
	motor->md_st = WAITING;
}
void FL_2_Motor_SetTimeSettings(MotorDefinition *motor)
{
	motor->md_FL2_startWorkTime = motor->md_FL2_Delay + GLB_Time[2];
	motor->md_FL2_stopWorkTime = motor->md_FL2_startWorkTime + motor->md_FL2_Time;
	motor->md_st = WAITING;
}
void FL_1_Motor_Start(MotorDefinition *motor)
{
	DRIVER_CTRL_ON;
	if(motor->md_st == WAITING)
	{
		HAL_TIM_PWM_Start(motor->md_htim, motor->md_chl);
		HAL_TIM_PWM_Start(motor->md_htim, motor->md_chr);
		motor->md_st = WORKING;
	}
}
void FL_2_Motor_Start(MotorDefinition *motor)
{
	DRIVER_CTRL_ON;
	motor->md_encod_sn.cnt = 0;
	Motor[0].md_CountDataToRecv = 0;
	motor->md_FL2_NowAngle = 0;
	EncTime[0] = 0;
	EncTimeNow[0] = 0;
	EncTimeOld[0] = 0;

	EncCnt[0] = 0;
	EncCntNow[0] = 0;
	EncCntOld[0] = 0;

	ContRegulatorValue = 0;

	if(motor->md_st == WAITING)
	{
		HAL_TIM_PWM_Start(motor->md_htim, motor->md_chl);
		HAL_TIM_PWM_Start(motor->md_htim, motor->md_chr);
		motor->md_st = WORKING;
	}
}
void FL_1_Motor_Stop(MotorDefinition *motor)
{
	HAL_TIM_PWM_Stop(motor->md_htim, motor->md_chl);
	HAL_TIM_PWM_Stop(motor->md_htim, motor->md_chr);

	motor->md_chl_value = 0;
	motor->md_chr_value = 0;

	motor->md_stParam.delay_cnfg = false;
	motor->md_stParam.dir_cnf = false;
	motor->md_stParam.pwm_cnfg = false;
	motor->md_stParam.timeWork_cnfg = false;
	motor->EnableADC = false;

	motor->md_workTime = 0;
	motor->md_delayTime = 0;
	motor->md_startWorkTime = 0;
	motor->md_stopWorkTime = 0;

	if(motor->EnableADC) motor->EnableADC = false;

	motor->md_st = FINISH;
}
void FL_2_Motor_Stop(MotorDefinition *motor)
{

	motor->md_FL2_startWorkTime = 0;
	motor->md_FL2_stopWorkTime = 0;
	motor->md_FL2_Angle = 0;
	motor->md_FL2_Delay = 0;
//	motor->md_FL2_NowAngle = 0;
//	motor->md_encod_sn.cnt = 0;
	motor->md_FL2_Speed = 0;
	motor->md_FL2_Time = 0;
	motor->md_encod_sn.cnt = 0;

	motor->md_stParam.fl2_angle = false;
	motor->md_stParam.fl2_time = false;
	motor->md_stParam.fl2_speed = false;
	motor->md_stParam.fl2_delay = false;

	FL_2_Motor_SetDuty(motor, 1000, 1000);

	motor->md_st = FINISH;
}

RCV_Flags Rcv_ChechFlags(uint8_t *package)
{
	RCV_Flags flags = {0, };
	uint16_t mask = 0x01;
	uint16_t allpack = ((package[1] << 8) | package[0]);
	uint16_t i = 0;
	i +=2;
	if (allpack & mask) // SidePlate
	{
		flags.FL0_SidePlate = true;
		i++;
	}
	mask <<= 1;
	if (allpack & mask)	// WorkMode byte
	{
		flags.FL0_WorkMode = true;
		i++;
	}
	mask <<= 1;
	if(package[3] == FL_PWM_MODE)
	{
		GLB_TypeCtrl = 0x01;
		if (allpack & mask) // SelectMotor byte
		{
			flags.FL1_MotorSelect = true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // TypeMoveMotor byte
		{
			flags.FL1_MotorDir= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // PWM byte
		{
			flags.FL1_PWM_Set= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // TimeWork byte
		{
			flags.FL1_TimeWork= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // DelayWork byte
		{
			flags.FL1_DelayWork= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // ADC byte
		{
			flags.FL1_ADC = true;
			if(package[i] == 0x01) ETEMode_Enable = true;				////////////////
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Start instruction
		{
			flags.FL0_StartInsruct = true;
			i++;
		}
		mask <<= 1;
	}
	else if (package[3] == FL_ANGLE_MODE)
	{
		GLB_TypeCtrl = 0x02;
		if (allpack & mask) // SelectMotor byte
		{
			flags.FL1_MotorSelect = true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // TypeMoveMotor byte
		{
			flags.FL1_MotorDir= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Angle byte
		{
			flags.FL2_Angle= true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Time byte
		{
			flags.FL2_Time = true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Speed byte
		{
			flags.FL2_Speed = true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Delay byte
		{
			flags.FL2_Delay = true;
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // FeedBack byte
		{
			flags.FL2_FeedBack = true;
			if(package[i] == 0x01) ETEMode_Enable = true;				////////////////
			i++;
		}
		mask <<= 1;
		if (allpack & mask) // Start instruction
		{
			flags.FL0_StartInsruct = true;
			i++;
		}
		mask <<= 1;
	}
	return flags;
}
uint8_t Rcv_FL_1_SelectMotor(uint8_t byte)
{
	return byte;
}
MotorMoveState Rcv_FL_1_SelectMotorDir(uint8_t byte, uint8_t num_mot)
{
	MotorMoveState dir_st;
	if(byte == 0x00) dir_st = FREE;
	else if(byte == 0x01) dir_st = LEFT;
	else if(byte == 0x02) dir_st = RIGHT;
	else if(byte == 0x03) dir_st = HOLD;
	else if(byte == 0x03) dir_st = ANGLE_MODE;

	Motor[num_mot].md_rotsd = dir_st;
	Motor[num_mot].md_stParam.dir_cnf = true;
	return dir_st;
}
void Rcv_FL_1_SetPwm(uint8_t byte, uint8_t num_motor, MotorMoveState dr)
{
	if(byte > 0x64) byte = 0x64;
	if(dr == FREE) FL_1_Motor_SetDuty(&Motor[num_motor], 0, 0);
	else if(dr == LEFT) FL_1_Motor_SetDuty(&Motor[num_motor], byte * 10, 0);
	else if(dr == RIGHT) FL_1_Motor_SetDuty(&Motor[num_motor], 0, byte * 10);
	else if(dr == HOLD) FL_1_Motor_SetDuty(&Motor[num_motor], byte * 10, byte * 10);
	if(byte != 0) Motor[num_motor].md_stParam.pwm_cnfg = true;
}
void Rcv_FL_1_SetTime(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_workTime = halfword;
	if(halfword != 0) Motor[num_motor].md_stParam.timeWork_cnfg = true;
}
void Rcv_FL_1_SetDelay(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_delayTime = halfword;
	if(halfword != 0) Motor[num_motor].md_stParam.delay_cnfg = true;
}
void Rcv_FL_1_SetADC(uint8_t byte, uint8_t num_motor)
{
	if(byte & 0x01)
	{
		Motor[num_motor].EnableADC = true;
		ProtezGlobalConf.ADC_ChannelsEnable = true;	// если хоть один установился
		NowCountPointADC = 1250;
	}
	else if (!(byte & 0x01))
	{
		Motor[num_motor].EnableADC = false;
		ProtezGlobalConf.ADC_ChannelsEnable = false;	// если хоть один установился
		NowCountPointADC = 0;
	}
}
void Rcv_FL_1_StartInstr(uint8_t byte)
{
	if(byte & 0x01) FL_1_HandProtezStartInstruction();
}

void FL_1_HandProtezStartInstruction(void)
{
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(Motor[i].md_stParam.pwm_cnfg && Motor[i].md_stParam.dir_cnf && Motor[i].md_stParam.timeWork_cnfg) Motor[i].md_st = CONFIGURATED;
		Target_dtsc[i] = (uint32_t)(2.0 * NowCountPointADC * (Motor[i].md_workTime / 100.0));
	}

	if(ProtezGlobalConf.ADC_ChannelsEnable)
	{
		PR_ADC_Init(NowCountPointADC);
	}
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(Motor[i].md_st == CONFIGURATED)
		{
			FL_1_Motor_SetTimeSettings(&Motor[i]);
		}
	}
}





uint8_t Rcv_FL_2_SelectMotor(uint8_t byte)
{
	return byte;
}
MotorMoveState Rcv_FL_2_SelectMotorDir(uint8_t byte, uint8_t num_mot)
{
	MotorMoveState dir_st;
	if(byte == 0x03) dir_st = HOLD;
	else if(byte == 0x04) dir_st = ANGLE_MODE;

	Motor[num_mot].md_rotsd = dir_st;
	Motor[num_mot].md_stParam.dir_cnf = true;
	return dir_st;
}
void Rcv_FL_2_SetAngle(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_FL2_Angle = halfword;
	if(halfword != 0)
	{

//		MotorMoveState dir_st;
//		if(Motor[num_motor].md_rotsd == ANGLE_MODE)
//		{
//
//		}
//
//		Motor[num_mot].md_rotsd = dir_st;
//		Motor[num_mot].md_stParam.dir_cnf = true;
//		return dir_st;


		Motor[num_motor].md_stParam.fl2_angle = true;
	}



}
void Rcv_FL_2_SetTime(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_FL2_Time = halfword;
	if(halfword != 0) Motor[num_motor].md_stParam.fl2_time = true;
}
void Rcv_FL_2_SetSpeed(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_FL2_Speed = halfword;
	if(halfword != 0) Motor[num_motor].md_stParam.fl2_speed = true;
}
void Rcv_FL_2_SetDelay(uint16_t halfword, uint8_t num_motor)
{
	Motor[num_motor].md_FL2_Delay = halfword;
	if(halfword != 0) Motor[num_motor].md_stParam.fl2_delay = true;
}
void Rcv_FL_2_SetFeedBack(uint8_t byte, uint8_t num_motor)
{
	if(byte & 0x01)
	{
		Motor[num_motor].md_EnableFeedBack = true;
		ProtezGlobalConf.FeedBack = true;	// если хоть один установился
	}
	else if (!(byte & 0x01))
	{
		Motor[num_motor].md_EnableFeedBack = false;
		ProtezGlobalConf.FeedBack = false;	// если хоть один установился
	}
}
void Rcv_FL_2_SetADC(uint8_t byte, uint8_t num_motor)
{
	if(byte & 0x01)
	{
		Motor[num_motor].EnableADC = true;
		ProtezGlobalConf.ADC_ChannelsEnable = true;	// если хоть один установился
		NowCountPointADC = 2500;
	}
	else if (!(byte & 0x01))
	{
		Motor[num_motor].EnableADC = false;
		ProtezGlobalConf.ADC_ChannelsEnable = false;	// если хоть один установился
		NowCountPointADC = 0;
	}
}
void Rcv_FL_2_StartInstr(uint8_t byte)
{
	if(byte & 0x01) FL_2_HandProtezStartInstruction();
}
void FL_2_HandProtezStartInstruction(void)
{
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{

		HAL_TIM_PWM_Stop(Motor[i].md_htim, Motor[i].md_chl);
		HAL_TIM_PWM_Stop(Motor[i].md_htim, Motor[i].md_chr);

		if((Motor[i].md_stParam.fl2_angle && Motor[i].md_stParam.fl2_time && Motor[i].md_stParam.fl2_speed))
		{
			// ERROR
		}
		else if ((Motor[i].md_stParam.fl2_angle && Motor[i].md_stParam.fl2_time))	// Angle-Time
		{
			TCM = ANGLE_TIME;
			/*Calculate first error in definition*/
//			FL_2_Motor_SetDuty(&Motor[i], 0, 60);

			if(Motor[i].md_stParam.fl2_angle && Motor[i].md_stParam.fl2_time) Motor[i].md_st = CONFIGURATED;

		}
		else if ((Motor[i].md_stParam.fl2_angle && Motor[i].md_stParam.fl2_speed))	// Angle-Speed
		{
			TCM = ANGLE_SPEED;
			if(Motor[i].md_stParam.pwm_cnfg && Motor[i].md_stParam.dir_cnf && Motor[i].md_stParam.timeWork_cnfg) Motor[i].md_st = CONFIGURATED;

		}
		else if ((Motor[i].md_stParam.fl2_time && Motor[i].md_stParam.fl2_speed))	// Time-Speed
		{
			TCM = TIME_SPEED;
			if(Motor[i].md_stParam.pwm_cnfg && Motor[i].md_stParam.dir_cnf && Motor[i].md_stParam.timeWork_cnfg) Motor[i].md_st = CONFIGURATED;

		}
	}

//	ccc1 = 60.0 / 1000.0;

//	if(ProtezGlobalConf.ADC_ChannelsEnable)
//	{
//		PR_ADC_Init(NowCountPointADC);
//	}
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(Motor[i].md_st == CONFIGURATED)
		{
			FL_2_Motor_SetTimeSettings(&Motor[i]);
		}
	}
}
// This fumction meeds for correction settings
void HandProtezRecvInstructionCorrectToReverse(uint8_t *package, uint32_t count)
{
	uint8_t SubPackNum = 0;
	RCV_Flags FlagsRecvInstTemp = Rcv_ChechFlags(package);
	SubPackNum += 2;

	ThisDeviceOnUsartCtrl = true;
	package[SubPackNum] = FL_CURRENT_PLATE;
	if(package[SubPackNum] == FL_CURRENT_PLATE)
	{
		SubPackNum++;
		if(package[SubPackNum] == FL_PWM_MODE)
		{
			SubPackNum++;
			if(FlagsRecvInstTemp.FL1_MotorSelect)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_MotorDir)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_PWM_Set)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_TimeWork)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_DelayWork)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_ADC)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL0_StartInsruct)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
		}
		else if(package[SubPackNum] == FL_ANGLE_MODE)
		{
			SubPackNum++;
			if(FlagsRecvInstTemp.FL1_MotorSelect)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL1_MotorDir)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL2_Angle)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL2_Time)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL2_Speed)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL2_Delay)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL2_FeedBack)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
			if(FlagsRecvInstTemp.FL0_StartInsruct)
			{
//				package[SubPackNum] = ...;
				SubPackNum++;
			}
		}
	}
}


void HandProtezRecvInstruction(uint8_t *package, uint32_t count)
{
	uint8_t SubPackNum = 0;
	uint8_t num_motor = 0;
	MotorMoveState dir = FREE;
	FlagsRecvInst = Rcv_ChechFlags(package);
	SubPackNum += 2;

	if(package[SubPackNum] == FL_CURRENT_PLATE)
	{
		SubPackNum++;
		if(package[SubPackNum] == FL_PWM_MODE)
		{
			SubPackNum++;
			if(FlagsRecvInst.FL1_MotorSelect)
			{
				num_motor = Rcv_FL_1_SelectMotor(package[SubPackNum]);
				Motor[num_motor].TOM = FL_PWM_MODE;
				SubPackNum++;
			}
			if(FlagsRecvInst.FL1_MotorDir)
			{
				dir = Rcv_FL_1_SelectMotorDir(package[SubPackNum], num_motor);
				SubPackNum++;
			}
			if(FlagsRecvInst.FL1_PWM_Set)
			{
				Rcv_FL_1_SetPwm(package[SubPackNum], num_motor,  dir);
				SubPackNum++;
			}
			if(FlagsRecvInst.FL1_TimeWork)
			{
				Rcv_FL_1_SetTime((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL1_DelayWork)
			{
				Rcv_FL_1_SetDelay((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL1_ADC)
			{
				Rcv_FL_1_SetADC(package[SubPackNum], num_motor);
				SubPackNum++;
			}
			if(FlagsRecvInst.FL0_StartInsruct)
			{
				Rcv_FL_1_StartInstr(package[SubPackNum]);
				SubPackNum++;
			}
		}
		else if(package[SubPackNum] == FL_ANGLE_MODE)
		{
			SubPackNum++;
			if(FlagsRecvInst.FL1_MotorSelect)
			{
				num_motor = Rcv_FL_2_SelectMotor(package[SubPackNum]);
				Motor[num_motor].TOM = FL_ANGLE_MODE;
				SubPackNum++;
			}
			if(FlagsRecvInst.FL1_MotorDir)
			{
				dir = Rcv_FL_2_SelectMotorDir(package[SubPackNum], num_motor);
				SubPackNum++;
			}
			if(FlagsRecvInst.FL2_Angle)
			{
				Rcv_FL_2_SetAngle((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL2_Time)
			{
				Rcv_FL_2_SetTime((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL2_Speed)
			{
				Rcv_FL_2_SetSpeed((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL2_Delay)
			{
				Rcv_FL_2_SetDelay((package[SubPackNum + 1] << 8) | (package[SubPackNum]), num_motor);
				SubPackNum += 2;
			}
			if(FlagsRecvInst.FL2_FeedBack)
			{
				Rcv_FL_2_SetFeedBack(package[SubPackNum], num_motor);
				SubPackNum++;
			}
			if(FlagsRecvInst.FL0_StartInsruct)
			{
				Rcv_FL_2_StartInstr(package[SubPackNum]);
				SubPackNum++;
			}
		}
	}
	else
	{
		//
		// ИНИЦИАЛИЗИРУЕМ РЕЖИМ СКВОЗНОЙ ПЕРЕДАЧИ ПО USART
		// НАДО РЕАЛИЗОВАТЬ: [[STM1][STM2]] С РАЗНЫМИ ПОРЦИЯМИ ОТПРАВЛЯТЬ ПО ОЧЕРЕДИ: STM1-STM2-STM1-STM2...
		//

		if(ETEMode_Enable == true)
		{

		}


		//
		char data[50] = {0, };
		for(uint8_t i = 0; i < count; i++) data[i + 1] = package[i];
		data[0] = ++count;
		HAL_UART_Transmit_IT(&huart6, (uint8_t*)&data, count);
	}
}

void HandProtezUSBConnectHandler(void)
{
	DeviceIsConnected = true;
	ThisDeviceOnUsartCtrl = false;
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_RXNE);
}

MotorState CheckStateAllMotor(void)
{
	uint8_t ii = 0;
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if ((Motor[i].md_st == FINISH) || (Motor[i].md_st == RELEASED))
		{
			ii++;
		}

	}
	if (ii == ProtezGlobalConf.NumMotorConfigured) return FINISH;
	else return NONE;
}
void Init_TimerMotor(void)
{
	/* Timer 1 init*/

	/* Timer 3 init*/

	/* Timer 4 init*/

	/* Timer 5 init*/

}
/*
nomps - number of measurements per second
num_ch - number of ADC channels
*/
void ADC_Timer2_Init(uint8_t num_ch, uint32_t nomps)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 50 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	if ((num_ch == 0) && (num_ch > MAX_ADC_CHANNEL)) num_ch = 1;
	if ((nomps != 0) && (nomps <= MAX_MESUR_POINT))
	{
		htim2.Init.Period = (((HAL_RCC_GetSysClockFreq() / (htim2.Init.Prescaler + 1)) / (nomps /** num_pack*/))) - 1;
	}

	drts = num_pack * num_ch;	// 10 bytes * number channels

	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}
/*
nomps - number of measurements per second
*/
void ADC_Setchannel(uint8_t cnt, uint32_t trg_channel)
{
	ADC_Channels[cnt] = trg_channel;
}
void PR_ADC_Init(uint32_t nomps)
{
	uint8_t num_ch = 0;
	_Bool MotorFlags[ProtezGlobalConf.NumMotorConfigured];
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(Motor[i].EnableADC)
		{
			MotorFlags[i] = true;
			num_ch++;
			ProtezGlobalConf.ADC_ChannelsEnable = true;
		}
		else
		{
			MotorFlags[i] = false;
		}
	}


	/*Initialization ADC*/
	ADC_ChannelConfTypeDef sConfig = {0};
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = num_ch;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/*------------------*/
	uint8_t rank = 1;
	for(uint8_t i = 0; i < ProtezGlobalConf.NumMotorConfigured; i++)
	{
		if(MotorFlags[i])
		{
			sConfig.Channel = ADC_Channels[i];
			sConfig.Rank = rank;
			sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
			rank++;
		}
	}

	ADC_Timer2_Init(num_ch, nomps);
}

void ProtezInit(void)
{
	/*Global variables*/
	ProtezGlobalConf.NumMotorConfigured = 6;
	/*Motor definition*/
	Motor[0] = Motor_Settings(&htim3, PAIRCHANNEL_2, true);		// CH1
	Motor[1] = Motor_Settings(&htim4, PAIRCHANNEL_1, false);	// CH2
	Motor[2] = Motor_Settings(&htim4, PAIRCHANNEL_2, false);	// CH3
	Motor[3] = Motor_Settings(&htim1, PAIRCHANNEL_1, false);	// CH4
	Motor[4] = Motor_Settings(&htim3, PAIRCHANNEL_1, false);	// CH5
	Motor[5] = Motor_Settings(&htim5, PAIRCHANNEL_1, false);	// CH6
	for(uint8_t i = 0; i < 6; i++) Motor[5].md_st = RELEASED;
	for(uint8_t i = 0; i < 6; i++) FL_1_Motor_SetDuty(&Motor[i], 0, 0);
	/**/
	Motor[0].md_htim->Instance->CCR3 = 0;
	Motor[0].md_htim->Instance->CCR4 = 0;
	Motor[1].md_htim->Instance->CCR1 = 0;
	Motor[1].md_htim->Instance->CCR2 = 0;
	Motor[2].md_htim->Instance->CCR3 = 0;
	Motor[2].md_htim->Instance->CCR4 = 0;
	Motor[3].md_htim->Instance->CCR1 = 0;
	Motor[3].md_htim->Instance->CCR2 = 0;
	Motor[4].md_htim->Instance->CCR1 = 0;
	Motor[4].md_htim->Instance->CCR2 = 0;
	Motor[5].md_htim->Instance->CCR1 = 0;
	Motor[5].md_htim->Instance->CCR2 = 0;


	/*ADC channels definition*/
	ADC_Setchannel(0, ADC_CHANNEL_5);
	ADC_Setchannel(1, ADC_CHANNEL_3);
	ADC_Setchannel(2, ADC_CHANNEL_4);
	ADC_Setchannel(3, ADC_CHANNEL_2);
	ADC_Setchannel(4, ADC_CHANNEL_7);
	ADC_Setchannel(5, ADC_CHANNEL_6);
	/*Encoder parameters*/
	Motor[0].md_encod_sn = Encoder[0]; 		// CH1
	Motor[1].md_encod_sn = Encoder[1]; 		// CH2
	Motor[2].md_encod_sn = Encoder[2]; 		// CH3
	Motor[3].md_encod_sn = Encoder[3]; 		// CH4
	Motor[4].md_encod_sn = Encoder[4]; 		// CH5
	Motor[5].md_encod_sn = Encoder[5]; 		// CH6
	/*Support pin encoder parameters*/
	Motor[0].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_1;
	Motor[0].md_encod_sn.PINsupSens = SUP_ENC_PIN_1;
	Motor[1].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_2;
	Motor[1].md_encod_sn.PINsupSens = SUP_ENC_PIN_2;
	Motor[2].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_3;
	Motor[2].md_encod_sn.PINsupSens = SUP_ENC_PIN_3;
	Motor[3].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_4;
	Motor[3].md_encod_sn.PINsupSens = SUP_ENC_PIN_4;
	Motor[4].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_5;
	Motor[4].md_encod_sn.PINsupSens = SUP_ENC_PIN_5;
	Motor[5].md_encod_sn.GPIOsupSens = SUP_ENC_GPIO_6;
	Motor[5].md_encod_sn.PINsupSens = SUP_ENC_PIN_6;


}
void StartMeasurement(void)
{
	glb_dstc = 0;
	PR_TIM2_ON;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Data, drts);
}
void StopMeasurement(void)
{
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_ADC_DeInit(&hadc1);
	PR_TIM2_OFF;
	HAL_TIM_Base_DeInit(&htim2);
	dstc = 0;
	FlagDMA_START = false;
}
