/*
 * CyberMed_HandCotrol.h
 *
 *  Created on: Mar 11, 2025
 *      Author: Roman
 */

#ifndef INC_HANDCOTROL_H_
#define INC_HANDCOTROL_H_


#include "stm32f4xx_hal.h"

#define PACKET_USB (uint8_t)20
#define MAX_MESUR_POINT	(uint32_t)5000					// Max measurement point
#define MAX_ADC_CHANNEL (uint8_t)6						// Max adc channels

#define DRIVER_CTRL_GPIO	GPIOA
#define DRIVER_CTRL_PIN		GPIO_PIN_15

#define DRIVER_CTRL_ON		HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_SET)
#define DRIVER_CTRL_OFF		HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_RESET)

#define PR_TIM11_ON HAL_TIM_Base_Start_IT(&htim11)
#define PR_TIM11_OFF HAL_TIM_Base_Stop_IT(&htim11)
#define PR_TIM2_ON HAL_TIM_Base_Start_IT(&htim2)
#define PR_TIM2_OFF HAL_TIM_Base_Stop_IT(&htim2)
#define PR_TIM10_ON HAL_TIM_Base_Start_IT(&htim10)
#define PR_TIM10_OFF HAL_TIM_Base_Stop_IT(&htim10)
#define PR_TIM9_ON	HAL_TIM_Base_Start_IT(&htim9)
#define PR_TIM9_OFF	HAL_TIM_Base_Stop_IT(&htim9)


#define ENC_GPIO_1	GPIOB
#define ENC_PIN_1	GPIO_PIN_10
#define ENC_GPIO_2	GPIOB
#define ENC_PIN_2	GPIO_PIN_14
#define ENC_GPIO_3	GPIOB
#define ENC_PIN_3	GPIO_PIN_13
#define ENC_GPIO_4	GPIOC
#define ENC_PIN_4	GPIO_PIN_5
#define ENC_GPIO_5	GPIOC
#define ENC_PIN_5	GPIO_PIN_3
#define ENC_GPIO_6	GPIOC
#define ENC_PIN_6	GPIO_PIN_1


#define SUP_ENC_GPIO_1	GPIOB
#define SUP_ENC_PIN_1	GPIO_PIN_2
#define SUP_ENC_GPIO_2	GPIOB
#define SUP_ENC_PIN_2	GPIO_PIN_15
#define SUP_ENC_GPIO_3	GPIOB
#define SUP_ENC_PIN_3	GPIO_PIN_12
#define SUP_ENC_GPIO_4	GPIOC
#define SUP_ENC_PIN_4	GPIO_PIN_4
#define SUP_ENC_GPIO_5	GPIOC
#define SUP_ENC_PIN_5	GPIO_PIN_2
#define SUP_ENC_GPIO_6	GPIOC
#define SUP_ENC_PIN_6	GPIO_PIN_0

#define FL_PWM_MODE		0x01
#define FL_ANGLE_MODE	0x02

#define FL_INTERN_PLATE	0x00
#define FL_EXTERN_PLATE	0x01

#define FL_CURRENT_PLATE 0x00	////////////////////////////////////// WARINING!!!!


#define	PID_MAX_VAL	1000
#define PID_MIN_VAL	50


/*
TIM1_CH1 - PA8
TIM1_CH2 - PA9

TIM3_CH1 - PB4
TIM3_CH2 - PB5

TIM4_CH1 - PB6
TIM4_CH2 - PB7

TIM4_CH3 - PB8
TIM4_CH4 - PB9

TIM5_CH1 - PA0
TIM5_CH2 - PA1

TIM3_CH3 - PB0
TIM3_CH4 - PB1
*/

typedef enum NumMotor
{
	MOTOR_1 = 0,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_5,
	MOTOR_6,

} NumMotor;
typedef enum State
{
	OFF = 0,
	ON,
} State;
typedef enum PairChannelOut
{
	PAIRCHANNEL_1 = 0, // - CH1, CH2
	PAIRCHANNEL_2,	   // - CH3, CH4
} PairChannelOut;
typedef enum ChannelOut
{
	CHANNEL_1 = TIM_CHANNEL_1,
	CHANNEL_2 = TIM_CHANNEL_2,
	CHANNEL_3 = TIM_CHANNEL_3,
	CHANNEL_4 = TIM_CHANNEL_4,
} ChannelOut;
typedef enum MoveState
{
	FREE = 0,
	LEFT,
	RIGHT,
	HOLD,
//	ANGLE_MODE,
} MoveState;

typedef enum MotorState
{
	RELEASED = 0,				// No working, no configurated
	CONFIGURATED,				// It configurated
	WORKING,					// In work
	WAITING,					// Waiting in line
	FINISH,						// Last instruction is finished
	NONE,
} MotorState;
typedef enum TypeOperationMode
{
    WRM_None = 0,
    WRM_PWM_MODE = 0x01,
    WRM_ANGLE_MODE = 0x02,
} TypeOperationMode;

//typedef struct MotorFlagParam
//{
//	_Bool TimeWorkEn;
//	_Bool ValueChannel;
//
//} MotorFlagParam;

typedef struct StatusParam
{
	_Bool pwm_cnfg;
	_Bool timeWork_cnfg;
	_Bool delay_cnfg;
	_Bool dir_cnf;

	_Bool fl2_angle;
	_Bool fl2_time;
	_Bool fl2_speed;
	_Bool fl2_delay;
} StatusParam;
typedef struct EncoderDefinition
{
	GPIO_TypeDef *GPIO;
	uint16_t PIN;
	GPIO_TypeDef *GPIOsup;
	uint16_t PINsup;



	MoveState side[2];
	MoveState SideNow;
	uint32_t CNT;

} EncoderDefinition;

typedef struct MotorDefinition
{
	TIM_HandleTypeDef *md_htim;		// TIM
	PairChannelOut md_prch;			// Pair channels
	ChannelOut md_chl;				// First out channel
	ChannelOut md_chr;				// Second out channel
	MoveState md_rotsd;		// State motor
	uint32_t md_chl_value;			// Value PWM count (CCRN)
	uint32_t md_chr_value;			// Value PWM count (CCRN)
	MotorState md_st;				// State now
	uint16_t md_workTime;			// Time work instruction
	uint16_t md_delayTime;			// Time delay before start work instruction
	uint16_t md_startWorkTime;		// Tick 0.01s for to start work
	uint16_t md_stopWorkTime;		// Tick 0.01s for to stop work
	EncoderDefinition Encoder;		// Motor`s Encoder
	_Bool EnableADC;				// Enable adc conversion

	StatusParam md_stParam;

	uint16_t md_FL2_Angle;
	uint16_t md_FL2_Time;
	uint16_t md_FL2_Speed;
	uint16_t md_FL2_Delay;

	uint16_t md_FL2_startWorkTime;			// Tick 0.01s for to start work
	uint16_t md_FL2_stopWorkTime;			// Tick 0.01s for to stop work

	uint32_t md_CountDataToRecv;
	uint32_t md_CountDataRecieved;

	double md_FL2_NowAngle;

	_Bool md_EnableFeedBack;

	uint32_t md_drum_cnt;			// Drum counter motor

	TypeOperationMode TOM;
	NumMotor md_NMotor;
	MoveState md_rotsd_now;

//	MotorFlagParam md_ConfigSettingsParam;	// Flags init parameters
} MotorDefinition;

typedef struct PRGlbDef
{
	uint8_t NumMotorConfigured;
	_Bool ADC_ChannelsEnable;
	_Bool FeedBack;

	uint8_t md_countMotorADCEnable[2];
} PRGlbDef;

typedef struct RCV_Flags
{
	_Bool FL0_WorkMode;
	_Bool FL0_StartInsruct;
	_Bool FL0_SidePlate;

	_Bool FL1_MotorSelect;
	_Bool FL1_MotorDir;
	_Bool FL1_PWM_Set;
	_Bool FL1_TimeWork;
	_Bool FL1_DelayWork;
	_Bool FL1_ADC;


	_Bool FL2_Angle;
	_Bool FL2_Time;
	_Bool FL2_Delay;
	_Bool FL2_Speed;
	_Bool FL2_FeedBack;

} RCV_Flags;
typedef enum FL2_TypeCtrlMove
{
	ANGLE_TIME = 0,
	ANGLE_SPEED,
	TIME_SPEED,
} FL2_TypeCtrlMove;
/*===================================================================*/
/*
 * This function sets basic parameters
 */
void ProtezInit(void);

MotorDefinition Motor_Settings(TIM_HandleTypeDef *htim, PairChannelOut prch,_Bool invert);
void FL_1_Motor_SetDuty(MotorDefinition *motor, uint32_t duty_l, uint32_t duty_r);
void FL_2_Motor_SetDuty(MotorDefinition *motor, uint32_t duty_l, uint32_t duty_r);
void FL_1_Motor_SetTimeSettings(MotorDefinition *motor);
void FL_2_Motor_SetTimeSettings(MotorDefinition *motor);
void Motor_SettingsEncoder(MotorDefinition *motor, GPIO_TypeDef *gpio, uint16_t gpio_pin);
void Motor_SettingsSupEncoder(MotorDefinition *motor, GPIO_TypeDef *gpio, uint16_t gpio_pin, _Bool sideInvert);
void HandProtezRecvInstructionCorrectToReverse(uint8_t *package, uint32_t count);
void HandProtezRecvInstruction(uint8_t *package, uint32_t count);
void PR_ADC_Init(uint32_t nomps);
void ADC_Setchannel(uint8_t cnt, uint32_t trg_channel);
void StartMeasurement(void);
void FL_1_Motor_Start(MotorDefinition *motor);
void FL_2_Motor_Start(MotorDefinition *motor);
void FL_1_Motor_Stop(MotorDefinition *motor);
void FL_2_Motor_Stop(MotorDefinition *motor);
void StopMeasurement(void);
void FL_1_HandProtezStartInstruction(void);
void FL_2_HandProtezStartInstruction(void);
MotorState CheckStateAllMotor(void);
void FL_1_Motor_ContinuousDuty(MotorDefinition *motor);
void FL_2_Motor_ContinuousDuty(MotorDefinition *motor);
void HandProtezUSBConnectHandler();
/*===================================================================*/
void ADC_Timer2_Init(uint8_t num_ch, uint32_t nomps);
MotorState CheckStateAllMotor(void);
RCV_Flags Rcv_ChechFlags(uint8_t *package);
/*===================================================================*/






#endif /* INC_HANDCOTROL_H_ */
