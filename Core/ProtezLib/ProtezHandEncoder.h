/*
 * ProtezHandEncoder.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Roman
 */

#ifndef PROTEZLIB_PROTEZHANDENCODER_H_
#define PROTEZLIB_PROTEZHANDENCODER_H_

#include <stdbool.h>
#include <utility>
#include <stdint.h>

#include "stm32f4xx_hal.h"



enum encSide
{
	enc_NoMove = 0,
	enc_Left,
	enc_Right,
	enc_ERROR,
};


class ProtezHandEncoder {
private:
	uint32_t counter;
	uint32_t timePoint;
	encSide side;

	GPIO_TypeDef *PORT;
	uint16_t PIN;

	GPIO_TypeDef *PORT_sup;
	uint16_t PIN_sup;

public:
	ProtezHandEncoder();
	~ProtezHandEncoder();

	/*Устанавливает значение счетчика*/
	void setCounter(uint32_t);
	/*Возвращает значени счетчика*/
	uint32_t getCounter();
	/*Устанавливает время точки измерения в us*/
	void setTimePoint(uint32_t);
	/*Возвращает время точки измерения в us*/
	uint32_t getTimePoint();
	/*Устанавливает сторону вращения энкодера*/
	void setSide(encSide);
	/*Возвращает сторону вращения энкодера*/
	encSide getSide();
	/*Устанавливает порты и выводы энкодера*/
	void setAllGPIO(GPIO_TypeDef*, uint16_t, GPIO_TypeDef*, uint16_t, GPIO_PinState);
	/*Возвращает порт опорного вывода энкодера*/
	GPIO_TypeDef* getPORTmain();
	/*Возвращает номер опорного вывода энкодера*/
	uint16_t getPINmain();
	/*Возвращает порт вспомогательного вывода энкодера*/
	GPIO_TypeDef* getPORsup();
	/*Возвращает номер вспомогательного вывода энкодера*/
	uint16_t getPINsup();
	/*Установка количества измеряемых точек в секунду*/
	static void setCntPoints(uint32_t);
	/*Возвращает количеств измеряемы точек в секунду*/
	static uint32_t getCntPoint();




	uint16_t DataSPEED[1000] = {0, };
	uint32_t indexDataSPEED = 0;
	GPIO_PinState PositiveSideRotate = GPIO_PIN_RESET;

	static uint32_t cntPoints;


};












#endif /* PROTEZLIB_PROTEZHANDENCODER_H_ */
