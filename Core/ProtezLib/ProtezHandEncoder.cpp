///*
// * ProtezHandEncoder.cpp
// *
// *  Created on: Aug 29, 2025
// *      Author: Roman
// */
//
//#include "ProtezHandEncoder.h"
//
//
//uint32_t ProtezHandEncoder::cntPoints = 500;
//_Bool ProtezHandEncoder::fl_Enable;
//uint8_t ProtezHandEncoder::ImpulsData[500];
//uint32_t ProtezHandEncoder::ImpulsDataCounter;
//
//ProtezHandEncoder::ProtezHandEncoder()
//{
//}
//
//ProtezHandEncoder::~ProtezHandEncoder()
//{
//}
//void ProtezHandEncoder::setCounter(uint32_t counter)
//{
//	this->counter = counter;
//}
//uint32_t ProtezHandEncoder::getCounter()
//{
//	return counter;
//}
//void ProtezHandEncoder::setTimePoint(uint32_t timePoint)
//{
//	this->timePoint = timePoint;
//}
//uint32_t ProtezHandEncoder::getTimePoint()
//{
//	return timePoint;
//}
//void ProtezHandEncoder::setSide(encSide side)
//{
//	this->side = side;
//}
//encSide ProtezHandEncoder::getSide()
//{
//	return side;
//}
//void ProtezHandEncoder::setAllGPIO(GPIO_TypeDef *PORT, uint16_t PIN, GPIO_TypeDef *PORT_sup, uint16_t PIN_sup, GPIO_PinState PositiveSideRotate)
//{
//	this->PORT = PORT;
//	this->PIN = PIN;
//	this->PORT_sup = PORT_sup;
//	this->PIN_sup = PIN_sup;
//	this->PositiveSideRotate = PositiveSideRotate;
//}
//GPIO_TypeDef* ProtezHandEncoder::getPORTmain()
//{
//	return PORT;
//}
//uint16_t ProtezHandEncoder::getPINmain()
//{
//	return PIN;
//}
//GPIO_TypeDef* ProtezHandEncoder::getPORsup()
//{
//	return PORT_sup;
//}
//
//uint16_t ProtezHandEncoder::getPINsup()
//{
//	return PIN_sup;
//}
//void ProtezHandEncoder::setEnable(_Bool _fl_Enable)
//{
//	fl_Enable = _fl_Enable;
//}
///*Возвращает разрешено ли работать с энкодером*/
//_Bool ProtezHandEncoder::getEnable()
//{
//	return fl_Enable;
//}
//
///************************STATIC************************/
//
//
//
