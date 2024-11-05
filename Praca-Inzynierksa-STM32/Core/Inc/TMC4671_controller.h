/*
 * TMC4671_controller.h
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */

#ifndef INC_TMC4671_CONTROLLER_H_
#define INC_TMC4671_CONTROLLER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "TMC4671_API.h"
#include "gpio.h"
#include "LinearRamp.h"

#define TORQUE_FLUX_MAX 	(int32_t)10000
#define POSITION_SCALE_MAX  (int32_t)65536

typedef struct {
	uint16_t startVoltage;
	uint16_t initWaitTime;
	uint16_t actualInitWaitTime;
	uint8_t initState;
	uint8_t initMode;
	uint16_t torqueMeasurementFactor;  // uint8_t.uint8_t
	uint32_t maximumCurrent;
	uint8_t motionMode;
	int32_t actualVelocityPT1;
	int64_t akkuActualVelocity;
	int16_t actualTorquePT1;
	int64_t akkuActualTorque;
	int16_t actualFluxPT1;
	int64_t akkuActualFlux;
	int32_t positionScaler;
	int32_t linearScaler;
	int16_t hall_phi_e_old;
	int16_t hall_phi_e_new;
	int16_t hall_actual_coarse_offset;
	uint16_t last_Phi_E_Selection;
	uint32_t last_UQ_UD_EXT;
	int16_t last_PHI_E_EXT;
	uint8_t enableVelocityFeedForward;
} MotorConfig;

typedef enum {
	DRIVER_DISABLE, DRIVER_ENABLE, DRIVER_USE_GLOBAL_ENABLE
} DriverState;

typedef enum {
	TMC_ERROR_NONE = 0x00,
	TMC_ERROR_GENERIC = 0x01,
	TMC_ERROR_FUNCTION = 0x02,
	TMC_ERROR_MOTOR = 0x08,
	TMC_ERROR_VALUE = 0x10,
	TMC_ERROR_CHIP = 0x40
} TMCError;

static MotorConfig motorConfig;
static TMC_LinearRamp rampGenerator;
static uint8_t actualMotionMode;
static int32_t lastRampTargetPosition;
static int32_t lastRampTargetVelocity;
static DriverState driverState;

static void TMC4671_controller_enableDriver(DriverState state) {
	if (state == DRIVER_DISABLE) {
		driverState = DRIVER_DISABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	} else {
		driverState = DRIVER_ENABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
	}
}

void TMC4671_controller_init()
{
	// configure ENABLE-PIN for TMC4671

	// setting SD_STP (DIO6) and SD_DIR (DIO7) to High-Z
//	HAL.IOs->config->reset(&HAL.IOs->pins->DIO6);
//	HAL.IOs->config->reset(&HAL.IOs->pins->DIO7);

	TMC4671_controller_enableDriver(DRIVER_ENABLE);

	// init motor config
	motorConfig.initWaitTime             	= 1000;
	motorConfig.startVoltage             	= 6000;
	motorConfig.initMode                 	= 0;
	motorConfig.hall_phi_e_old				= 0;
	motorConfig.hall_phi_e_new				= 0;
	motorConfig.hall_actual_coarse_offset	= 0;
	motorConfig.last_Phi_E_Selection		= 0;
	motorConfig.last_UQ_UD_EXT				= 0;
	motorConfig.last_PHI_E_EXT				= 0;
	motorConfig.torqueMeasurementFactor  	= 256;
	motorConfig.maximumCurrent				= 1000;
	motorConfig.actualVelocityPT1			= 0;
	motorConfig.akkuActualVelocity       	= 0;
	motorConfig.actualTorquePT1				= 0;
	motorConfig.akkuActualTorque         	= 0;
	motorConfig.positionScaler				= POSITION_SCALE_MAX;
	motorConfig.enableVelocityFeedForward 	= true;
	motorConfig.linearScaler             	= 30000; // µm / rotation

	// set default polarity for evaluation board's power stage on init
	tmc4671_writeRegister(TMC4671_PWM_POLARITIES, 0x0);
	tmc4671_writeRegister(TMC4671_PWM_SV_CHOP, TMC4671_PWM_SV_MASK);	// enable space vector PWM by default
	tmc4671_writeRegister(TMC4671_PWM_BBM_H_BBM_L, 0x00001919);

	tmc4671_writeRegister(TMC4671_DSADC_MCLK_B, 0x0);

	// set default acceleration and max velocity
	tmc4671_writeRegister(TMC4671_PID_VELOCITY_LIMIT, 4000);

	// set default max torque/flux
	tmc4671_setTorqueFluxLimit_mA(motorConfig.torqueMeasurementFactor, motorConfig.maximumCurrent);

	// init ramp generator
	TMC_LinearRamp_init(&rampGenerator);
	actualMotionMode = TMC4671_MOTION_MODE_STOPPED;
	lastRampTargetPosition = 0;
	lastRampTargetVelocity = 0;

	// update ramp generator default values
	rampGenerator.maxVelocity = (uint32_t)tmc4671_readRegister(TMC4671_PID_VELOCITY_LIMIT);
	rampGenerator.acceleration = 2000;
}

void TMC4671_controller_deInit(void) {
	TMC4671_controller_enableDriver(DRIVER_DISABLE);
}

uint32_t TMC4671_controller_rotate(int32_t velocity) {
	// switch to velocity motion mode
	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_VELOCITY);

	// set target velocity for ramp generator
	rampGenerator.targetVelocity = velocity;

	// remember switched motion mode
	actualMotionMode = TMC4671_MOTION_MODE_VELOCITY;

	return TMC_ERROR_NONE;
}

uint32_t TMC4671_controller_moveTo(int32_t position) {
	// scale target position
	position = (float) position * (float) POSITION_SCALE_MAX
			/ (float) motorConfig.positionScaler;

	// switch to position motion mode
	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	rampGenerator.targetPosition = position;

	// remember switched motion mode
	actualMotionMode = TMC4671_MOTION_MODE_POSITION;

	return TMC_ERROR_NONE;
}

uint32_t TMC4671_controller_moveBy(int32_t *ticks) {
	// scale position deviation
	int32_t dX = (float) *ticks * (float) POSITION_SCALE_MAX
			/ (float) motorConfig.positionScaler;

	// switch to position motion mode
	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);

	// set target position for ramp generator
	rampGenerator.targetPosition = tmc4671_readRegister(
			TMC4671_PID_POSITION_ACTUAL) + dX;

	// remember switched motion mode
	actualMotionMode = TMC4671_MOTION_MODE_POSITION;

	return TMC_ERROR_NONE;
}

uint8_t TMC4671_controller_positionReached(int32_t targetPosition,
		int32_t actualPosition, int32_t actualVelocity, int32_t maxPosDiff,
		int32_t maxVel) {
	if ((labs((long) targetPosition - (long) actualPosition) <= maxPosDiff)
			& (labs((long) actualVelocity) <= maxVel)) {
		return 1;
	}
	return 0;
}

void TMC4671_controller_periodicJob(void) {
    tmc4671_periodicJob(motorConfig.initMode, &(motorConfig.initState), motorConfig.initWaitTime,
                        &(motorConfig.actualInitWaitTime), motorConfig.startVoltage,
                        &(motorConfig.hall_phi_e_old), &(motorConfig.hall_phi_e_new),
                        &(motorConfig.hall_actual_coarse_offset),
                        &(motorConfig.last_Phi_E_Selection), &(motorConfig.last_UQ_UD_EXT),
                        &(motorConfig.last_PHI_E_EXT));

    // 1ms velocity ramp handling
    motorConfig.actualVelocityPT1 = tmc_filterPT1( &motorConfig.akkuActualVelocity, tmc4671_getActualVelocity(),
												   motorConfig.actualVelocityPT1, 3, 8);

	// filter actual current
	int32_t
	actualCurrentRaw = tmc4671_fieldRead(TMC4671_PID_TORQUE_ACTUAL_FIELD);
	if ((actualCurrentRaw > -32000) && (actualCurrentRaw < 32000)) {
		int32_t actualCurrent = ((int32_t) actualCurrentRaw
				* (int32_t) motorConfig.torqueMeasurementFactor) / 256;
		motorConfig.actualTorquePT1 = tmc_filterPT1(
				&motorConfig.akkuActualTorque, actualCurrent,
				motorConfig.actualTorquePT1, 4, 8);
	}

	// filter actual flux
	int32_t
	actualFluxRaw = tmc4671_fieldRead(TMC4671_PID_FLUX_ACTUAL_FIELD);
	if ((actualFluxRaw > -32000) && (actualFluxRaw < 32000)) {
		int32_t actualFlux = ((int32_t) actualFluxRaw
				* (int32_t) motorConfig.torqueMeasurementFactor) / 256;
		motorConfig.actualFluxPT1 = tmc_filterPT1(
				&motorConfig.akkuActualFlux, actualFlux,
				motorConfig.actualFluxPT1, 2, 8);
	}
	// do velocity / position ramping
	if (actualMotionMode == TMC4671_MOTION_MODE_POSITION) {
		TMC_LinearRamp_computeRampPosition(&rampGenerator);

		// set new target position (only if changed)
		if (rampGenerator.rampPosition != lastRampTargetPosition) {
			tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
					rampGenerator.rampPosition);
			lastRampTargetPosition = rampGenerator.rampPosition;

			// use velocity feed forward
			tmc4671_writeRegister(TMC4671_PID_VELOCITY_OFFSET,
					(motorConfig.enableVelocityFeedForward) ?
							rampGenerator.rampVelocity : 0);
		}

		// sync ramp velocity by PIDIN_TARGET_VELOCITY if ramp is disabled
		if (!rampGenerator.rampEnabled) {
			rampGenerator.rampVelocity = tmc4671_readFieldWithDependency(TMC4671_PIDIN_TARGET_VELOCITY_FIELD, TMC4671_INTERIM_ADDR, 2);
		}
	}
	else if (actualMotionMode == TMC4671_MOTION_MODE_VELOCITY) {
		TMC_LinearRamp_computeRampVelocity(&rampGenerator);

		// set new target velocity (only if changed)
		if (rampGenerator.rampVelocity != lastRampTargetVelocity) {
			// set new target velocity
			tmc4671_writeRegister(TMC4671_PID_VELOCITY_TARGET,
					rampGenerator.rampVelocity);
			lastRampTargetVelocity = rampGenerator.rampVelocity;

			// turn of velocity feed forward
			tmc4671_writeRegister(TMC4671_PID_VELOCITY_OFFSET, 0);
		}

		// keep position ramp and target position on track
		tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
				tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL));
		rampGenerator.rampPosition = tmc4671_readRegister(
				TMC4671_PID_POSITION_ACTUAL);
		rampGenerator.lastdXRest = 0;
	}
	else if (actualMotionMode == TMC4671_MOTION_MODE_TORQUE) {
		// keep position ramp and target position on track
		tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
				tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL));
		rampGenerator.rampPosition = tmc4671_readRegister(
				TMC4671_PID_POSITION_ACTUAL);
		rampGenerator.rampVelocity = tmc4671_getActualVelocity();
		rampGenerator.lastdXRest = 0;
	}

}

#endif /* INC_TMC4671_CONTROLLER_H_ */
