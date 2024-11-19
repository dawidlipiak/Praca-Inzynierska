/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#include <TMC4671_API.h>

//int32_t tmc4671_fieldRead(RegisterField field)
//{
//    int32_t value = tmc4671_readRegister(field.address);
//    return tmc4671_fieldExtract(value, field);
//}
//
//int32_t tmc4671_fieldUpdate(int32_t data, RegisterField field, int32_t value)
//{
//    return (data & (~field.mask)) | ((value << field.shift) & field.mask);
//}
//
//void tmc4671_fieldWrite(RegisterField field, int32_t value)
//{
//    int32_t regValue = tmc4671_readRegister(field.address);
//    regValue = tmc4671_fieldUpdate(regValue, field, value);
//    tmc4671_writeRegister(field.address, regValue);
//}

void tmc4671_switchToMotionMode(uint8_t mode)
{
    tmc4671_fieldWrite(TMC4671_MODE_MOTION_FIELD, mode);
}

void tmc4671_setTargetTorque_raw(int32_t targetTorque)
{
    tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_TORQUE);
    tmc4671_fieldWrite(TMC4671_PID_TORQUE_TARGET_FIELD, targetTorque);
}

int32_t tmc4671_getTargetTorque_raw()
{
    uint32_t lastIndex = tmc4671_readRegister(TMC4671_INTERIM_ADDR);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, 0);
    int32_t value = tmc4671_readRegister(TMC4671_INTERIM_DATA);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, lastIndex);
    return value;
}

int32_t tmc4671_getActualTorque_raw()
{
    return tmc4671_fieldRead(TMC4671_PID_TORQUE_ACTUAL_FIELD);
}

void tmc4671_setTargetTorque_mA(uint16_t torqueMeasurementFactor, int32_t targetTorque)
{
    tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_TORQUE);
    tmc4671_fieldWrite(TMC4671_PID_TORQUE_TARGET_FIELD, (targetTorque * 256) / (int32_t)torqueMeasurementFactor);
}

int32_t tmc4671_getTargetTorque_mA(uint16_t torqueMeasurementFactor)
{
    return (tmc4671_getTargetTorque_raw() * (int32_t)torqueMeasurementFactor) / 256;
}

int32_t tmc4671_getActualTorque_mA(uint16_t torqueMeasurementFactor)
{
    return (tmc4671_getActualTorque_raw() * (int32_t)torqueMeasurementFactor) / 256;
}

int32_t tmc4671_getTargetTorqueFluxSum_mA(uint16_t torqueMeasurementFactor)
{
    uint32_t lastIndex = tmc4671_readRegister(TMC4671_INTERIM_ADDR);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, 0);
    int32_t torque = tmc4671_readRegister(TMC4671_INTERIM_DATA);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, 1);
    int32_t flux = tmc4671_readRegister(TMC4671_INTERIM_DATA);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, lastIndex);
    return ((flux + torque) * (int32_t)torqueMeasurementFactor) / 256;
}

int32_t tmc4671_getActualTorqueFluxSum_mA(uint16_t torqueMeasurementFactor)
{
    int32_t registerValue = tmc4671_readRegister(TMC4671_PID_TORQUE_FLUX_ACTUAL);
    int16_t flux = (registerValue & 0xFFFF);
    int16_t torque = ((registerValue >> 16) & 0xFFFF);
    return ((flux + torque) * (int32_t)torqueMeasurementFactor) / 256;
}

void tmc4671_setTargetFlux_raw(int32_t targetFlux)
{
    tmc4671_fieldWrite(TMC4671_PID_FLUX_TARGET_FIELD, targetFlux);
}

int32_t tmc4671_getTargetFlux_raw()
{
    uint32_t lastIndex = tmc4671_readRegister(TMC4671_INTERIM_ADDR);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, 1);
    int32_t value = tmc4671_readRegister(TMC4671_INTERIM_DATA);
    tmc4671_writeRegister(TMC4671_INTERIM_ADDR, lastIndex);
    return value;
}

int32_t tmc4671_getActualFlux_raw()
{
    return tmc4671_fieldRead(TMC4671_PID_FLUX_ACTUAL_FIELD);
}

void tmc4671_setTargetFlux_mA(uint16_t torqueMeasurementFactor, int32_t targetFlux)
{
    tmc4671_fieldWrite(TMC4671_PID_FLUX_TARGET_FIELD, (targetFlux * 256) / (int32_t)torqueMeasurementFactor);
}

int32_t tmc4671_getTargetFlux_mA(uint16_t torqueMeasurementFactor)
{
    return (tmc4671_getTargetFlux_raw() * (int32_t)torqueMeasurementFactor) / 256;
}

int32_t tmc4671_getActualFlux_mA(uint16_t torqueMeasurementFactor)
{
    return (tmc4671_getActualFlux_raw() * (int32_t)torqueMeasurementFactor) / 256;
}

void tmc4671_setTorqueFluxLimit_mA(uint16_t torqueMeasurementFactor, int32_t max)
{
    tmc4671_fieldWrite(TMC4671_PID_TORQUE_FLUX_LIMITS_FIELD, (max * 256) / (int32_t)torqueMeasurementFactor);
}

int32_t tmc4671_getTorqueFluxLimit_mA(uint16_t torqueMeasurementFactor)
{
    return (tmc4671_fieldRead(TMC4671_PID_TORQUE_FLUX_LIMITS_FIELD) * (int32_t)torqueMeasurementFactor) / 256;
}

void tmc4671_setTargetVelocity(int32_t targetVelocity)
{
    tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_VELOCITY);
    tmc4671_writeRegister(TMC4671_PID_VELOCITY_TARGET, targetVelocity);
}

int32_t tmc4671_getTargetVelocity()
{
    return tmc4671_readRegister(TMC4671_PID_VELOCITY_TARGET);
}

int32_t tmc4671_getActualVelocity()
{
    return tmc4671_readRegister(TMC4671_PID_VELOCITY_ACTUAL);
}

void tmc4671_setAbsolutTargetPosition(int32_t targetPosition)
{
    tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);
    tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET, targetPosition);
}

void tmc4671_setRelativeTargetPosition(int32_t relativePosition)
{
    tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);
    tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET, tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL) + relativePosition);
}

int32_t tmc4671_getTargetPosition()
{
    return tmc4671_readRegister(TMC4671_PID_POSITION_TARGET);
}

void tmc4671_setActualPosition(int32_t actualPosition)
{
    tmc4671_writeRegister(TMC4671_PID_POSITION_ACTUAL, actualPosition);
}

int32_t tmc4671_getActualPosition()
{
    return tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL);
}

void tmc4671_doEncoderInitializationMode0(uint8_t *initState, uint16_t initWaitTime, uint16_t *actualInitWaitTime, uint16_t startVoltage,
    uint16_t *last_Phi_E_Selection, uint32_t *last_UQ_UD_EXT, int16_t *last_PHI_E_EXT)
{
    switch (*initState)
    {
    case STATE_NOTHING_TO_DO:
        *actualInitWaitTime = 0;
        break;
    case STATE_START_INIT:
        *last_Phi_E_Selection = (uint16_t)tmc4671_fieldRead(TMC4671_PHI_E_SELECTION_FIELD);
        *last_UQ_UD_EXT = (uint32_t)tmc4671_readRegister(TMC4671_UQ_UD_EXT);
        *last_PHI_E_EXT = (int16_t)tmc4671_fieldRead(TMC4671_PHI_E_EXT_FIELD);

        tmc4671_writeRegister(TMC4671_MODE_RAMP_MODE_MOTION, TMC4671_MOTION_MODE_UQ_UD_EXT);
        tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, 0);
        tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, 1);
        tmc4671_fieldWrite(TMC4671_UQ_EXT_FIELD, 0);
        tmc4671_fieldWrite(TMC4671_UD_EXT_FIELD, startVoltage);
        tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, 0);

        *initState = STATE_WAIT_INIT_TIME;
        break;
    case STATE_WAIT_INIT_TIME:
        (*actualInitWaitTime)++;
        if (*actualInitWaitTime >= initWaitTime)
        {
            tmc4671_writeRegister(TMC4671_ABN_DECODER_COUNT, 0);
            tmc4671_writeRegister(TMC4671_UQ_UD_EXT, *last_UQ_UD_EXT);
            tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, *last_PHI_E_EXT);
            tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, *last_Phi_E_Selection);
            *initState = STATE_ESTIMATE_OFFSET;
        }
        break;
    case STATE_ESTIMATE_OFFSET:
        *initState = 0;
        break;
    default:
        *initState = 0;
        break;
    }
}

int16_t tmc4671_getS16CircleDifference(int16_t newValue, int16_t oldValue)
{
	return (newValue - oldValue);
}

void tmc4671_doEncoderInitializationMode2(uint8_t *initState, uint16_t *actualInitWaitTime,
		int16_t *hall_phi_e_old, int16_t *hall_phi_e_new, int16_t *hall_actual_coarse_offset, uint16_t *last_Phi_E_Selection)
{
	switch (*initState)
	{
	case STATE_NOTHING_TO_DO:
		*actualInitWaitTime = 0;
		break;
	case STATE_START_INIT: // started by writing 1 to initState
		// save actual set value for PHI_E_SELECTION
		*last_Phi_E_Selection = (uint16_t)tmc4671_fieldRead(TMC4671_PHI_E_SELECTION_FIELD);

		// turn hall_mode interpolation off (read, clear bit 8, write back)
		tmc4671_writeRegister(TMC4671_HALL_MODE, tmc4671_readRegister(TMC4671_HALL_MODE) & 0xFFFFFEFF);

		// set ABN_DECODER_PHI_E_OFFSET to zero
		tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, 0);

		// read actual hall angle
		*hall_phi_e_old = tmc4671_fieldRead(TMC4671_HALL_PHI_E_FIELD);

		// read actual abn_decoder angle and compute difference to actual hall angle
		*hall_actual_coarse_offset = tmc4671_getS16CircleDifference(*hall_phi_e_old, (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD));

		// set ABN_DECODER_PHI_E_OFFSET to actual hall-abn-difference, to use the actual hall angle for coarse initialization
		tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, *hall_actual_coarse_offset);

		// normally MOTION_MODE_UQ_UD_EXT is only used by e.g. a wizard, not in normal operation
		if (tmc4671_fieldRead(TMC4671_MODE_MOTION_FIELD) != TMC4671_MOTION_MODE_UQ_UD_EXT)
		{
			// select the use of phi_e_hall to start icID with hall signals
			tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, TMC4671_PHI_E_HALL);
		}

		*initState = STATE_WAIT_INIT_TIME;
		break;
	case STATE_WAIT_INIT_TIME:
		// read actual hall angle
		*hall_phi_e_new = tmc4671_fieldRead(TMC4671_HALL_PHI_E_FIELD);

		// wait until hall angle changed
		if(*hall_phi_e_old != *hall_phi_e_new)
		{
			// estimated value = old value + diff between old and new (handle int16_t overrun)
			int16_t hall_phi_e_estimated = *hall_phi_e_old + tmc4671_getS16CircleDifference(*hall_phi_e_new, *hall_phi_e_old)/2;

			// read actual abn_decoder angle and consider last set abn_decoder_offset
			int16_t abn_phi_e_actual = (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD) - *hall_actual_coarse_offset;

			// set ABN_DECODER_PHI_E_OFFSET to actual estimated angle - abn_phi_e_actual difference
			tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, tmc4671_getS16CircleDifference(hall_phi_e_estimated, abn_phi_e_actual));

			// switch back to last used PHI_E_SELECTION setting
			tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, *last_Phi_E_Selection);

			// go to ready state
			*initState = 0;
		}
		break;
	default:
		*initState = 0;
		break;
	}
}

// analog encoder initialization
void tmc4671_doEncoderInitializationMode3(uint8_t *initState, uint16_t initWaitTime, uint16_t *actualInitWaitTime, uint16_t startVoltage,
		uint16_t *last_Phi_E_Selection, uint32_t *last_UQ_UD_EXT, int16_t *last_PHI_E_EXT)
{
	switch (*initState)
	{
	case STATE_NOTHING_TO_DO:
		*actualInitWaitTime = 0;
		break;
	case STATE_START_INIT: // started by writing 1 to initState

		// save actual set values for PHI_E_SELECTION, UQ_UD_EXT, and PHI_E_EXT
		*last_Phi_E_Selection = (uint16_t)tmc4671_fieldRead(TMC4671_PHI_E_SELECTION_FIELD);
		*last_UQ_UD_EXT = (uint32_t)tmc4671_readRegister(TMC4671_UQ_UD_EXT);
		*last_PHI_E_EXT = (int16_t)tmc4671_fieldRead(TMC4671_PHI_E_EXT_FIELD);

		// switch motion mode for running motor in open loop
		tmc4671_writeRegister(TMC4671_MODE_RAMP_MODE_MOTION, TMC4671_MOTION_MODE_UQ_UD_EXT);

		// set AENC_DECODER_PHI_E_PHI_M_OFFSET and AENC_DECODER_PHI_A_OFFSET to zero
		tmc4671_writeRegister(TMC4671_AENC_DECODER_PHI_E_PHI_M_OFFSET, 0);
		tmc4671_fieldWrite(TMC4671_AENC_DECODER_PHI_A_OFFSET_FIELD, 0);

		// select phi_e_ext
		tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, 1);

		// set an initialization voltage on UD_EXT (to the flux, not the torque!)
		tmc4671_fieldWrite(TMC4671_UQ_EXT_FIELD, 0);
		tmc4671_fieldWrite(TMC4671_UD_EXT_FIELD, startVoltage);

		// set the "zero" angle
		tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, 0);

		*initState = STATE_WAIT_INIT_TIME;
		break;
	case STATE_WAIT_INIT_TIME:
		// wait until initialization time is over (until no more vibration on the motor)
		(*actualInitWaitTime)++;
		if(*actualInitWaitTime >= initWaitTime)
		{
            // save actual DECODER_PHI_M as -DECODER_PHI_M_OFFSET
			int16_t offset_raw = tmc4671_fieldRead(TMC4671_AENC_DECODER_PHI_M_FIELD);

            // update PHI_M and keep PHI_E and PH_A at zero
            tmc4671_fieldWrite(TMC4671_AENC_DECODER_PHI_E_OFFSET_FIELD, 0);

            tmc4671_fieldWrite(TMC4671_AENC_DECODER_PHI_M_OFFSET_FIELD, -offset_raw);
            tmc4671_fieldWrite(TMC4671_AENC_DECODER_PHI_A_OFFSET_FIELD, 0);

			// switch back to last used UQ_UD_EXT setting
			tmc4671_writeRegister(TMC4671_UQ_UD_EXT, *last_UQ_UD_EXT);

			// set PHI_E_EXT back to last value
			tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, *last_PHI_E_EXT);

			// switch back to last used PHI_E_SELECTION setting
			tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, *last_Phi_E_Selection);

			// go to next state
			*initState = STATE_ESTIMATE_OFFSET;
		}
		break;
	case STATE_ESTIMATE_OFFSET:
		// you can do offset estimation here (wait for N-Channel if available and save encoder value)

		// go to ready state
		*initState = 0;
		break;
	default:
		*initState = 0;
		break;
	}
}


void tmc4671_checkEncderInitialization(uint8_t initMode, uint8_t *initState, uint16_t initWaitTime, uint16_t *actualInitWaitTime, uint16_t startVoltage,
                                       int16_t *hall_phi_e_old, int16_t *hall_phi_e_new, int16_t *hall_actual_coarse_offset,
                                       uint16_t *last_Phi_E_Selection, uint32_t *last_UQ_UD_EXT, int16_t *last_PHI_E_EXT)
{
    // Realizowanie inicjalizacji enkodera w zależności od trybu
    if (initMode == 0) {
        tmc4671_doEncoderInitializationMode0(initState, initWaitTime, actualInitWaitTime, startVoltage, last_Phi_E_Selection, last_UQ_UD_EXT, last_PHI_E_EXT);
    }
    else if (initMode == 3) { // analog encoder initialization
        tmc4671_doEncoderInitializationMode3(initState, initWaitTime, actualInitWaitTime, startVoltage, last_Phi_E_Selection, last_UQ_UD_EXT, last_PHI_E_EXT);
    }

    // Tryb bez konieczności timera
    if (initMode == 2) {
        tmc4671_doEncoderInitializationMode2(initState, actualInitWaitTime, hall_phi_e_old, hall_phi_e_new, hall_actual_coarse_offset, last_Phi_E_Selection);
    }
}

void tmc4671_periodicJob(uint8_t initMode, uint8_t *initState, uint16_t initWaitTime, uint16_t *actualInitWaitTime, uint16_t startVoltage,
                         int16_t *hall_phi_e_old, int16_t *hall_phi_e_new, int16_t *hall_actual_coarse_offset,
                         uint16_t *last_Phi_E_Selection, uint32_t *last_UQ_UD_EXT, int16_t *last_PHI_E_EXT)
{
    tmc4671_checkEncderInitialization(initMode, initState, initWaitTime, actualInitWaitTime, startVoltage,
                                      hall_phi_e_old, hall_phi_e_new, hall_actual_coarse_offset, last_Phi_E_Selection, last_UQ_UD_EXT, last_PHI_E_EXT);
}

void tmc4671_startEncoderInitialization(uint8_t mode, uint8_t *initMode, uint8_t *initState)
{
	// allow only a new initialization if no actual initialization is running
	if(*initState == STATE_NOTHING_TO_DO)
	{
		if(mode == 0) // estimate offset
		{
			// set mode
			*initMode = 0;

			// start initialization
			*initState = 1;
		}
		else if(mode == 2) // use hall sensor signals
		{
			// set mode
			*initMode = 2;

			// start initialization
			*initState = 1;
		}
		else if (mode == 3) // use analog encoder initialization
		{
			// set mode
			*initMode = 3;

			// start initialization
			*initState = 1;
		}
	}
}

int32_t tmc4671_readFieldWithDependency(RegisterField field, uint8_t dependsReg, uint32_t dependsValue)
{
	// remember old depends value
	uint32_t lastDependsValue = tmc4671_readRegister(dependsReg);

	// set needed depends value
	tmc4671_writeRegister(dependsReg, dependsValue);
	uint32_t value = tmc4671_fieldRead(field);

	// set old depends value
	tmc4671_writeRegister(dependsReg, lastDependsValue);
	return value;
}
