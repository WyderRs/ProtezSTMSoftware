/*
 * ProtezHandADC.cpp
 *
 *  Created on: Jul 29, 2025
 *      Author: makar
 */


#include "ProtezHandADC.h"

/*TO PC FORMAT DATA*/

uint8_t ProtezHandADC::DataADC[500];

extern PrHand_Motor_typedef Motor[6];

uint8_t data2[10];

ProtezHandADC::ProtezHandADC(ADC_HandleTypeDef* _adc, TIM_HandleTypeDef* _tim) : Adc(_adc), Tim(_tim)
{
	Adc = _adc;
	Tim = _tim;
}
ProtezHandADC::~ProtezHandADC()
{}
void ProtezHandADC::setNumberChannels(uint32_t numCh)
{
	if (numberChannels >= PR_ADC_MAX_CHANNEL) numberChannels = PR_ADC_MAX_CHANNEL;
	else numberChannels = numCh;
}
uint8_t ProtezHandADC::getNumberChannels()
{
	return numberChannels;
}
void ProtezHandADC::setPoints(uint32_t pt)
{
	if (pt >= PR_ADC_MAX_POINT) Points = PR_ADC_MAX_POINT;
	else Points = pt;
}
uint32_t ProtezHandADC::getPoints()
{
	return Points;
}

void ProtezHandADC::ADC_TimerInit()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	Tim->Instance = TIM2;
	Tim->Init.Prescaler = 50 - 1;
	Tim->Init.CounterMode = TIM_COUNTERMODE_UP;

	Tim->Init.Period = (((HAL_RCC_GetSysClockFreq() / (Tim->Init.Prescaler + 1)) / (Points))) - 1;

//	drts = num_pack * num_ch;	// 20 bytes * number channels

	Tim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Tim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(Tim) != HAL_OK) Error_Handler();
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(Tim, &sClockSourceConfig) != HAL_OK) Error_Handler();
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(Tim, &sMasterConfig) != HAL_OK) Error_Handler();
}
void ProtezHandADC::ADC_Init()
{
	std::vector<PrHand_Motor_typedef> temp;

	ConfiguredChannels = 0;

	for (auto motor : Motor) {
		if (motor.getEnabledADC() == _ADC_Motor_Enable) {
			temp.push_back(motor);
			ConfiguredChannels++;
		}
	}

	/*Initialization ADC*/
	ADC_ChannelConfTypeDef sConfig = {0};
	Adc->Instance = ADC1;
	Adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	Adc->Init.Resolution = ADC_RESOLUTION_12B;
	Adc->Init.ScanConvMode = ENABLE;
	Adc->Init.ContinuousConvMode = DISABLE;
	Adc->Init.DiscontinuousConvMode = DISABLE;
	Adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	Adc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	Adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	Adc->Init.NbrOfConversion = ConfiguredChannels;
	Adc->Init.DMAContinuousRequests = ENABLE;
	Adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(Adc) != HAL_OK)
	{
		Error_Handler();
	}
	/*------------------*/
	uint8_t rank = 1;
	for (auto motor : temp)
	{
		sConfig.Channel = motor.getADCChannel();
		sConfig.Rank = rank;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(Adc, &sConfig) != HAL_OK) Error_Handler();
		rank++;
	}

	ADC_TimerInit();
	setStateADC(_ADC_Configured);
}
void ProtezHandADC::setConfiguredChannels(uint8_t cfgCh)
{
	ConfiguredChannels = cfgCh;
}
uint8_t ProtezHandADC::getConfiguredChannels()
{
	return ConfiguredChannels;
}
void ProtezHandADC::setPackSizeData(uint32_t size)
{
	/*УСТАНОВИТЬ ОГРАНИЧЕНИЯ ПО РАЗМЕРУ ПАЧКИ*/
	PackSize = size;
}
uint32_t ProtezHandADC::getPackSizeData()
{
	return PackSize;
}


void ProtezHandADC::StartADC()
{
	HAL_TIM_Base_Start_IT(Tim);
	HAL_ADC_Start_DMA(Adc, (uint32_t*)&DataADC, PackSize);

	GLB_ADC_state = _ADC_Working;
}
void ProtezHandADC::StopADC()
{
	HAL_ADC_Stop(Adc);
	HAL_ADC_Stop_DMA(Adc);
	HAL_ADC_DeInit(Adc);
	HAL_TIM_Base_DeInit(Tim);

	GLB_ADC_state = _ADC_Released;
}
void ProtezHandADC::setStateADC(PrHand_GLB_ADCState st)
{
	GLB_ADC_state = st;
}
PrHand_GLB_ADCState ProtezHandADC::getEnabledADC()
{
	for (auto &motor : Motor) {
		if (motor.getEnabledADC() == _ADC_Motor_Enable)  return GLB_ADC_state = _ADC_NoConfigured;
	}
	return GLB_ADC_state = _ADC_Disable;
}
PrHand_GLB_ADCState ProtezHandADC::getStateADC()
{
	return GLB_ADC_state;
}



