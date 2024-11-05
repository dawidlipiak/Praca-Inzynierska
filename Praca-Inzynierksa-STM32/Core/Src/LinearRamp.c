/*
 * ramp.cpp
 *
 *  Created on: Oct 29, 2024
 *      Author: dawid
 */

#include "LinearRamp.h"


void TMC_LinearRamp_init(TMC_LinearRamp *ramp)
{
	ramp->maxVelocity     = 0;
	ramp->targetPosition  = 0;
	ramp->targetVelocity  = 0;
	ramp->rampVelocity    = 0;
	ramp->acceleration    = 0;
	ramp->encoderSteps	= UINT16_MAX;
	ramp->lastdVRest      = 0;
	ramp->lastdXRest      = 0;
	ramp->rampEnabled     = false;
}

void TMC_LinearRamp_computeRampVelocity(TMC_LinearRamp *ramp)
{
	if (ramp->rampEnabled)
	{
		// update target velocity according actual set acceleration
		// (scaling pre-factor of 1000 used for 1ms velocity ramp handling)

		int32_t dV = ramp->acceleration;

		// to ensure that small velocity changes at high set acceleration are also possible
		int32_t maxDTV = abs(ramp->targetVelocity - ramp->rampVelocity);
		if (maxDTV < (dV/1000))
			dV = maxDTV*1000;

		dV += ramp->lastdVRest;
		ramp->lastdVRest = dV % 1000;

		if (ramp->rampVelocity < ramp->targetVelocity)
		{
			// accelerate motor
			ramp->rampVelocity += dV/1000;	// divide with pre-factor
		}
		else if (ramp->rampVelocity > ramp->targetVelocity)
		{
			// decelerate motor
			ramp->rampVelocity -= dV/1000;	// divide with pre-factor
		}
	}
	else
	{
		// use target velocity directly
		ramp->rampVelocity = ramp->targetVelocity;
	}

	// limit ramp velocity
	ramp->rampVelocity = limitU32(ramp->rampVelocity, ramp->maxVelocity, ramp->maxVelocity);
}

void TMC_LinearRamp_computeRampPosition(TMC_LinearRamp *ramp)
{
	if (ramp->rampEnabled)
	{
		// update target position according actual set acceleration and max velocity
		// (scaling pre-factor of 1000 used for 1ms position ramp handling)

		// limit position difference for further computations
		int32_t targetPositionsDifference = ramp->targetPosition-ramp->rampPosition;

		// limit the sqrti value in case of high position differences
		int64_t sqrtiValue = limitS64(((int64_t)120 * (int64_t)ramp->acceleration * (int64_t)(abs(targetPositionsDifference))) / (int64_t)ramp->encoderSteps, 0, (int64_t)ramp->maxVelocity*(int64_t)ramp->maxVelocity);

		// compute max allowed ramp velocity to ramp down to target
		int32_t maxRampStop = sqrt_int(sqrtiValue);

		// compute max allowed ramp velocity
		int32_t maxRampTargetVelocity = 0;
		if (targetPositionsDifference > 0)
		{
			maxRampTargetVelocity = limitU32(maxRampStop, 0, (int32_t)ramp->maxVelocity);
		}
		else if (targetPositionsDifference < 0)
		{
			maxRampTargetVelocity = limitU32(-maxRampStop, -(int32_t)ramp->maxVelocity, 0);
		}
		else
		{
			//maxRampTargetVelocity = 0;
		}

		int32_t dV = ramp->acceleration;  // pre-factor ~ 1/1000

		// to ensure that small velocity changes at high set acceleration are also possible
		int32_t maxDTV = abs(maxRampTargetVelocity - ramp->rampVelocity);
		if (maxDTV < (dV / 1000))
			dV = maxDTV * 1000;

		dV += ramp->lastdVRest;
		ramp->lastdVRest = dV % 1000;

		// do velocity ramping
		if (maxRampTargetVelocity > ramp->rampVelocity)
		{
			ramp->rampVelocity += dV / 1000;
		}
		else if (maxRampTargetVelocity < ramp->rampVelocity)
		{
			ramp->rampVelocity -= dV / 1000;
		}

		// do position ramping using actual ramp velocity to update dX
		int64_t dX = ((int64_t)ramp->rampVelocity * (int64_t)ramp->encoderSteps) / ((int64_t)60) + ramp->lastdXRest;

		// scale actual target position
		int64_t tempActualTargetPosition = (int64_t)ramp->rampPosition * 1000;

		// reset helper variables if ramp position reached target position
		if (abs(ramp->targetPosition - ramp->rampPosition) < abs((int32_t)dX/1000))
		{
			// sync ramp position with target position on small deviations
			ramp->rampPosition = ramp->targetPosition;

			// update actual target position
			tempActualTargetPosition = (int64_t)ramp->rampPosition * 1000;

			dX = 0;
			ramp->lastdXRest = 0;
			ramp->rampVelocity = 0;
		}
		else
		{
			// update actual target position
			tempActualTargetPosition += dX;
		}

		int64_t absTempActualTargetPosition = (tempActualTargetPosition >= 0) ? tempActualTargetPosition : -tempActualTargetPosition;

		if (tempActualTargetPosition >= 0)
			ramp->lastdXRest = (absTempActualTargetPosition % 1000);
		else if (tempActualTargetPosition < 0)
			ramp->lastdXRest = -(absTempActualTargetPosition % 1000);

		// scale actual target position back
		ramp->rampPosition = tempActualTargetPosition / 1000;
	}
	else
	{
		// use target position directly
		ramp->rampPosition = ramp->targetPosition;

		// hold ramp velocity in reset
		ramp->rampVelocity = 0;
	}
}


