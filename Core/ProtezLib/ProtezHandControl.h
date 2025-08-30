#ifndef INC_PROTEZHANDCONTROL_H_
#define INC_PROTEZHANDCONTROL_H_

#ifdef __cplusplus

#include <stdbool.h>
#include <utility>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "ProtezHandUsbProtocol.h"
#include "ProtezHandADC.h"
#include "ProtezHandEncoder.h"




#define DRIVER_CTRL_GPIO	GPIOA
#define DRIVER_CTRL_PIN		GPIO_PIN_15

#define DRIVER_CTRL_ON		HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_SET)
#define DRIVER_CTRL_OFF		HAL_GPIO_WritePin(DRIVER_CTRL_GPIO, DRIVER_CTRL_PIN, GPIO_PIN_RESET)


#define PR_MAX_PWM 1000
#define PR_MIN_PWM 0




enum prHand_Direction
{
	PR_DIR_NONE = 0,
	PR_DIR_FORWARD,
	PR_DIR_BACKFORWARD,
};
enum PrHand_MotCh
{
	PR_CHANNEL_1 = 0,
	PR_CHANNEL_2,
	PR_CHANNEL_3,
	PR_CHANNEL_4,
};

enum PrHand_MoveState
{
	PR_MS_Stop = 0,
	PR_MS_Left,
	PR_MS_Right,
	PR_MS_Hold,
};
enum PrHand_MotorSetState
{
	_NoConfigured = 0,
	_Configured,
	_Launched,
	_Working,
	_Ending,
	_Released,
	_Error,
};
enum PrHand_MotorADCState
{
	_ADC_Motor_Disable = 0,
	_ADC_Motor_Enable,
};

enum PrHand_ADC_Channels
{
	ADC_Channel_1 = ADC_CHANNEL_1, ADC_Channel_2 = ADC_CHANNEL_2,
	ADC_Channel_3 = ADC_CHANNEL_3, ADC_Channel_4 = ADC_CHANNEL_4,
	ADC_Channel_5 = ADC_CHANNEL_5, ADC_Channel_6 = ADC_CHANNEL_6,
	ADC_Channel_7 = ADC_CHANNEL_7, ADC_Channel_8 = ADC_CHANNEL_8,
	ADC_Channel_9 = ADC_CHANNEL_9, ADC_Channel_10 = ADC_CHANNEL_10,
	ADC_Channel_11 = ADC_CHANNEL_11, ADC_Channel_12 = ADC_CHANNEL_12,
};


class PrHand_Motor_typedef
{
private:
	uint8_t id;
    TIM_HandleTypeDef *TIM;

    static TIM_HandleTypeDef* GLBTimer;
    static uint32_t GLBTimerCounter;
    uint32_t CH[2];
    volatile uint32_t *CCR[2];
    prHand_Direction GPIO_Dir;		// Прямое или обратное направление выводов каналов


    PrHand_MoveState targetSide;	// Нарпавление заданное командой
    uint32_t PWM;					// Скважность ШИМ
    uint32_t CNT_WorkTime;			// Время работы
    uint32_t CNT_WorkDelay;			// Время задержки перед началом работы
    uint32_t CNT_WorkTimeBegin;		// Время начала работы
    uint32_t CNT_WorkTimeEnd;		// Время завершения работы

    PrHand_ADC_Channels ADC_Channel;
    PrHand_MotorADCState ADC_MotorState;
    PrHand_MotorSetState state;
    PrHand_MoveState currentSide;
public:
	PrHand_Motor_typedef(uint8_t _id);
    ~PrHand_Motor_typedef();

    uint8_t getID();


    void setTIM(TIM_HandleTypeDef*);
    _Bool setChannel(std::pair<PrHand_MotCh, PrHand_MotCh>, prHand_Direction);
    void setPWM(uint32_t);
    void setTargetSide(PrHand_MoveState);
    void setWorkTime(uint16_t);
    void setWorkDelay(uint16_t);


    void setADCChannel(PrHand_ADC_Channels);					/*Устанавливает канал АЦП для этого двигателя.*/
    PrHand_ADC_Channels getADCChannel();						/*ВОзвращает установленный канал АЦП на двигателе.*/

    void enableADC();								/*Разрешение АЦП.*/
    void disableADC();								/*Запрет АЦП.*/
    PrHand_MotorADCState getEnabledADC();			/*Возвращает состояние ацп*/

    void Start();									/*Запуск двигателя.*/
    void Stop();									/*Остановка двигателя.*/
    void setParameters();							/*Установка параметров времени двигателя.*/
    void setState(PrHand_MotorSetState);			/*Установка состояния двигателя.*/

    /*STATIC FUNCTION*/
//    												/*Инициализация АЦП.*/
//    static void StartADC();						/*Запуск АЦП.*/
//    static void StopADC();						/*Остановка АЦП.*/
//    static void setStateADC(PrHand_GLB_ADCState);	/*Установка состояния АЦП.*/
//    static PrHand_GLB_ADCState getStateADC();		/*Возвращает текущее состояние АЦП.*/

    static void SetTIMHandlerInstr(TIM_HandleTypeDef*);		/*Установка глобального таймера.*/
    static void StartTIMHandlerInstr();						/*Запуск глобального таймера.*/
    static void StopTIMHandlerInstr();						/*Остановка глобального таймера.*/
    static void setTIMHandlerInstrCounter(uint32_t);			/*Установка значения глобального таймера.*/
    static uint32_t getTIMHandlerInstrCounter();				/*Возвращается занчение глобального таймера.*/



    ProtezHandEncoder cls_encoder;					/*Класс энкодера*/
    std::pair<PrHand_MotCh, PrHand_MotCh> getChannel(); /*Возвращает пару номеров каналов.*/
    uint32_t getPWM();			/*Возвращает текущий ШИМ.*/
    uint32_t getWorkTime();		/*Возвращает установленное время работы.*/
    uint32_t getWorkDelay();	/*Возвращает установленное время задеркжи.*/
    PrHand_MoveState getTargetSide();	/*Возвращает установленное направление вращения.*/

    PrHand_MotorSetState getState();		 /*Возвращает текущее состояние двигателя.*/
    PrHand_MotorSetState CheckWorkInterval();/*Проверяет интервал работы. Когда нужно запустить двигатель и когда его остановить.*/

    static uint32_t MaxTimeInterval;


private:
    void insert_pwm();						 /*Устанавливает ШИМ и направление.*/
};






#endif // __cplusplus

#endif /* INC_PROTEZHANDCONTROL_H_ */
