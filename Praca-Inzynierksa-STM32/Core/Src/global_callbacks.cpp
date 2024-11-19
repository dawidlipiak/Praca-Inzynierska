/*
 * global_callbacks.cpp
 *
 *  Created on: Nov 18, 2024
 *      Author: dawid
 */

#include "vector"
#include <global_callbacks.h>
#include "main.h"
#include "constants.h"

//
//#ifdef ADC1_CHANNELS
//volatile uint32_t ADC1_BUF[ADC1_CHANNELS] = {0};
//extern ADC_HandleTypeDef hadc1;
//#endif
//#ifdef ADC2_CHANNELS
//volatile uint32_t ADC2_BUF[ADC2_CHANNELS] = {0};
//extern ADC_HandleTypeDef hadc2;
//#endif
//#ifdef ADC3_CHANNELS
//volatile uint32_t ADC3_BUF[ADC3_CHANNELS] = {0};
//extern ADC_HandleTypeDef hadc3;
//#endif
//
///**
// * Callback after an adc finished conversion
// */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	//Pulse braking mosfet if internal voltage is higher than supply.
//	if(hadc == &VSENSE_HADC)
//		brakeCheck();
//
//	uint8_t chans = 0;
//	volatile uint32_t* buf = getAnalogBuffer(hadc,&chans);
//	if(buf == NULL)
//		return;
//
//	for(AdcHandler* c : AdcHandler::adcHandlers){
//		c->adcUpd(buf,chans,hadc);
//	}
//}
//
//volatile uint32_t* getAnalogBuffer(ADC_HandleTypeDef* hadc,uint8_t* chans){
//	#ifdef ADC1_CHANNELS
//	if(hadc == &hadc1){
//		*chans = ADC1_CHANNELS;
//		return ADC1_BUF;
//	}
//	#endif
//
//	#ifdef ADC2_CHANNELS
//	if(hadc == &hadc2){
//		*chans = ADC2_CHANNELS;
//		return ADC2_BUF;
//	}
//	#endif
//
//	#ifdef ADC3_CHANNELS
//	if(hadc == &hadc3){
//		*chans = ADC3_CHANNELS;
//		return ADC3_BUF;
//	}
//	#endif
//	return NULL;
//}
//
//void startADC(){
//	#ifdef ADC1_CHANNELS
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_BUF, ADC1_CHANNELS);
//	#endif
//	#ifdef ADC2_CHANNELS
//	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC2_BUF, ADC2_CHANNELS);
//	#endif
//	#ifdef ADC3_CHANNELS
//	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC3_BUF, ADC3_CHANNELS);
//	#endif
//}
