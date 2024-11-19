/*
 * global_callbacks.h
 *
 *  Created on: Nov 18, 2024
 *      Author: dawid
 */

#ifndef INC_GLOBAL_CALLBACKS_H_
#define INC_GLOBAL_CALLBACKS_H_

//#include "main.h"
//
//#pragma once
//#ifdef __cplusplus
//
//#include <vector>
//
///**
// * Helper function to add an instance pointer to a callback vector
// * Will check if it is already present and return with no change
// */
//template <class C> void addCallbackHandler(std::vector<C>& vec, C instance){
//	for(uint8_t i = 0; i < vec.size(); i++){
//		if( (vec)[i] == instance)
//			return;
//	}
//	vec.push_back(instance);
//}
//
///**
// * Helper function to remove an instance pointer from a callback vector
// */
//template <class C> void removeCallbackHandler(std::vector<C>& vec, C instance){
//	for (uint8_t i = 0; i < vec.size(); i++){
//		if( (vec)[i] == instance){
//			vec.erase(vec.begin()+i);
//			break;
//		}
//	}
//}
//
//extern "C" {
//#endif
//
//void startADC();
//volatile uint32_t* getAnalogBuffer(ADC_HandleTypeDef* hadc,uint8_t* chans); // Returns the DMA buffer for a hadc reference
//
//#ifdef __cplusplus
//}
//#endif
//
#endif /* INC_GLOBAL_CALLBACKS_H_ */
