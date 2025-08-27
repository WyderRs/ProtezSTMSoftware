/*
 * ProtezHandADC.h
 *
 *  Created on: Jul 29, 2025
 *      Author: makar
 */

#ifndef PROTEZLIB_PROTEZHANDADC_H_
#define PROTEZLIB_PROTEZHANDADC_H_

#ifdef __cplusplus

#include "ProtezHandControl.h"
#include "stm32f4xx_hal.h"
#include <utility>
#include <vector>


#define PR_ADC_MAX_POINT	(uint32_t)5000 	// Максимальное число точек за 1 с.
#define PR_ADC_MAX_CHANNEL 	(uint8_t)6		// Максимальное число каналов




enum PrHand_GLB_ADCState
{
	_ADC_NoConfigured = 0,
	_ADC_Configured,
	_ADC_Working,
	_ADC_Released,
	_ADC_Disable,
};

class ProtezHandADC {
private:
	ADC_HandleTypeDef* Adc;
	TIM_HandleTypeDef* Tim;
	uint8_t numberChannels;
	uint8_t ConfiguredChannels;
	uint32_t Points;		// Число точек за 1 с измерений.

	uint32_t PackSize;
    PrHand_GLB_ADCState GLB_ADC_state = _ADC_Disable;
public:
	ProtezHandADC(ADC_HandleTypeDef*, TIM_HandleTypeDef*);
	~ProtezHandADC();

	void ADC_TimerInit();
	void ADC_Init();

	void setNumberChannels(uint32_t);
	uint8_t getNumberChannels();

	void setPoints(uint32_t);
	uint32_t getPoints();

	void setConfiguredChannels(uint8_t);
	uint8_t getConfiguredChannels();

	void setPackSizeData(uint32_t);
	uint32_t getPackSizeData();


	/*Инициализация АЦП.*/
	void StartADC();							/*Запуск АЦП.*/
	void StopADC();								/*Остановка АЦП.*/
	void setStateADC(PrHand_GLB_ADCState);	/*Установка состояния АЦП.*/
	PrHand_GLB_ADCState getStateADC();		/*Возвращает текущее состояние АЦП.*/
	PrHand_GLB_ADCState getEnabledADC();	/*Возвращает разрешено ли АЦП.*/
public:
//	static std::vector<uint8_t> DataADC;
	static uint8_t DataADC[500];
};

#endif /*__cplusplus*/
#endif /* PROTEZLIB_PROTEZHANDADC_H_ */
