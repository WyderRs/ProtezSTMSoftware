#ifndef INC_PROTEZHANDCONTROL_H_
#define INC_PROTEZHANDCONTROL_H_

#ifdef __cplusplus

#include <stdbool.h>
#include <utility>
#include <stdint.h>

#include "stm32f4xx_hal.h"


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

class PrHand_Motor_typedef
{
private:
    TIM_HandleTypeDef *TIM;
    uint32_t CH[2];
    volatile uint32_t *CCR[2];
    prHand_Direction GPIO_Dir;

    uint32_t PWM;

    PrHand_MoveState targetSide;
    PrHand_MoveState currentSide;


public:
	PrHand_Motor_typedef();
    ~PrHand_Motor_typedef();


    void setTIM(TIM_HandleTypeDef*);
    _Bool setChannel(std::pair<PrHand_MotCh, PrHand_MotCh>, prHand_Direction);
    void setPWM(uint32_t);
    void setPWMCounter(uint32_t);
    void setTargetSide(PrHand_MoveState);


    std::pair<PrHand_MotCh, PrHand_MotCh> getChannel();
    uint32_t getPWM();
    uint32_t getPWMCounter();


    void Start();
    void Stop();


    static void StartTIM(TIM_HandleTypeDef*);
    static void StopTIM(TIM_HandleTypeDef*);

private:
    void insert_pwm();
};






#endif // __cplusplus

#endif /* INC_PROTEZHANDCONTROL_H_ */
