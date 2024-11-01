/*
 * TMC4671_controller.h
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */

#ifndef INC_TMC4671_CONTROLLER_HPP_
#define INC_TMC4671_CONTROLLER_HPP_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "TMC4671_API.h"
#include "gpio.h"
#include "LinearRamp.h"

#define TORQUE_FLUX_MAX 	(int32_t)10000
#define POSITION_SCALE_MAX  (int32_t)65536

typedef struct
{
	uint16_t  startVoltage;
	uint16_t  initWaitTime;
	uint16_t  actualInitWaitTime;
	uint8_t   initState;
	uint8_t   initMode;
	uint16_t  torqueMeasurementFactor;  // uint8_t.uint8_t
	uint32_t  maximumCurrent;
	uint8_t	  motionMode;
	int32_t   actualVelocityPT1;
	int64_t	  akkuActualVelocity;
	int16_t   actualTorquePT1;
	int64_t   akkuActualTorque;
	int16_t   actualFluxPT1;
	int64_t   akkuActualFlux;
	int32_t   positionScaler;
	int32_t   linearScaler;
	int16_t   hall_phi_e_old;
	int16_t   hall_phi_e_new;
	int16_t   hall_actual_coarse_offset;
	uint16_t  last_Phi_E_Selection;
	uint32_t  last_UQ_UD_EXT;
	int16_t   last_PHI_E_EXT;
	uint8_t	  enableVelocityFeedForward;
} MotorConfig;

typedef enum {
	DRIVER_DISABLE,
	DRIVER_ENABLE,
	DRIVER_USE_GLOBAL_ENABLE
} DriverState;

typedef enum {
	TMC_ERROR_NONE      = 0x00,
	TMC_ERROR_GENERIC   = 0x01,
	TMC_ERROR_FUNCTION  = 0x02,
	TMC_ERROR_MOTOR     = 0x08,
	TMC_ERROR_VALUE     = 0x10,
	TMC_ERROR_CHIP      = 0x40
} TMCError;

class TMC4671{
public:
	TMC4671();
	~TMC4671();

	void init(void);
	void deInit(void);

	uint32_t rotate(int32_t velocity);
	uint32_t moveTo(int32_t position);
	uint32_t moveBy(int32_t *ticks);
	uint8_t positionReached(int32_t targetPosition, int32_t actualPosition, int32_t actualVelocity, int32_t maxPosDiff, int32_t maxVel);
	void enableDriver(DriverState state);
	static void periodicJob(uint32_t actualSystick);


private:
	MotorConfig motorConfig;
	TMC_LinearRamp rampGenerator;
	uint8_t actualMotionMode;
	int32_t lastRampTargetPosition;
	int32_t lastRampTargetVelocity;
	DriverState driverState = DRIVER_DISABLE;
	int32_t tmc_filterPT1(int64_t *akku, int32_t newValue, int32_t lastValue, uint8_t actualFilter, uint8_t maxFilter);
};


#endif /* INC_TMC4671_CONTROLLER_HPP_ */
