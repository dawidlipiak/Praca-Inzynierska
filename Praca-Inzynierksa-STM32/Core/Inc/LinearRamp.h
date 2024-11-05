/*
 * LinearRamp.h
 *
 *  Created on: Oct 29, 2024
 *      Author: dawid
 */

#ifndef INC_LINEARRAMP_H_
#define INC_LINEARRAMP_H_

#include <stdint.h>
#include <stdlib.h>
#include "stdbool.h"
#include "util_functions.h"

typedef struct {
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
} TMC_LinearRamp;

// Function prototypes that work with TMC_LinearRamp instances
void TMC_LinearRamp_init(TMC_LinearRamp *ramp);
void TMC_LinearRamp_computeRampVelocity(TMC_LinearRamp *ramp);
void TMC_LinearRamp_computeRampPosition(TMC_LinearRamp *ramp);



#endif /* INC_LINEARRAMP_H_ */
