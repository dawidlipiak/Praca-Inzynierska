/*
 * LinearRamp.h
 *
 *  Created on: Oct 29, 2024
 *      Author: dawid
 */

#ifndef INC_LINEARRAMP_H_
#define INC_LINEARRAMP_H_

#include <cstdint>
#include <stdlib.h>

#define false 0;
#define true 1;

class TMC_LinearRamp {
public:
	uint32_t maxVelocity;
	int32_t targetPosition;
	int32_t rampPosition;
	int32_t targetVelocity;
	int32_t rampVelocity;
	int32_t acceleration;
	uint16_t encoderSteps;
	int32_t lastdVRest;
	int32_t lastdXRest;
	uint8_t rampEnabled;

	void init();
	void computeRampVelocity();
	void computeRampPosition();
};



#endif /* INC_LINEARRAMP_H_ */
