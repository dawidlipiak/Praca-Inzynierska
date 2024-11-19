/*******************************************************************************
* Copyright © 2016 TRINAMIC Motion Control GmbH & Co. KG
* (now owned by Analog Devices Inc.),
*
* Copyright © 2024 Analog Devices Inc. All Rights Reserved.
* This software is proprietary to Analog Devices, Inc. and its licensors.
*******************************************************************************/

#ifndef TMC_IC_TMC4671_API_H_
#define TMC_IC_TMC4671_API_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
//#include "util_functions.h"
#include "TMC4671_HW_Abstraction.h"
#include "gpio.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

#define TMC4671_WRITE_BIT 0x80
#define TMC4671_ADDRESS_MASK 0x7F

#define STATE_NOTHING_TO_DO    0
#define STATE_START_INIT       1
#define STATE_WAIT_INIT_TIME   2
#define STATE_ESTIMATE_OFFSET  3

// spi access
static uint32_t tmc4671_readRegister(uint8_t address)
{
    uint8_t txBuf[5] = { 0 };
    uint8_t rxBuf[5];

    // clear write bit
    txBuf[0] = TMC4671_ADDRESS_MASK & address;

    HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port,SPI1_SS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 5, 200);
    HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port,SPI1_SS1_Pin, GPIO_PIN_SET);

    uint32_t ret;
	memcpy(&ret, &rxBuf[1], 4);  // Bajty danych zaczynają się od rxBuf[1]
	ret = __REV(ret);  // Konwersja do little-endian

	return ret;
}

static void tmc4671_writeRegister(uint8_t address, uint32_t value)
{
    uint8_t data[5] = { 0 };

    data[0] = TMC4671_WRITE_BIT | address;

    value =__REV(value);
	memcpy(data+1,&value,4);

    HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port,SPI1_SS1_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, data, 5, 200);
    HAL_GPIO_WritePin(SPI1_SS1_GPIO_Port,SPI1_SS1_Pin, GPIO_PIN_SET);
}

static uint32_t tmc4671_fieldExtract(uint32_t data, RegisterField field)
{
    uint32_t value = (data & field.mask) >> field.shift;

    if (field.isSigned)
    {
        uint32_t baseMask = field.mask >> field.shift;
        uint32_t signMask = baseMask & (~baseMask >> 1);
        value = (value ^ signMask) - signMask;
    }

    return value;
}

static inline uint32_t tmc4671_fieldRead(RegisterField field)
{
    uint32_t value = tmc4671_readRegister(field.address);
    return tmc4671_fieldExtract(value, field);
}

static inline uint32_t tmc4671_fieldUpdate(uint32_t data, RegisterField field, uint32_t value)
{
    return (data & (~field.mask)) | ((value << field.shift) & field.mask);
}

static inline void tmc4671_fieldWrite(RegisterField field, uint32_t value)
{
    uint32_t regValue = tmc4671_readRegister(field.address);
    regValue = tmc4671_fieldUpdate(regValue, field, value);
    tmc4671_writeRegister(field.address, regValue);
}



// Do cyclic tasks
void tmc4671_periodicJob(uint8_t initMode, uint8_t *initState, uint16_t initWaitTime, uint16_t *actualInitWaitTime, uint16_t startVoltage,
                         int16_t *hall_phi_e_old, int16_t *hall_phi_e_new, int16_t *hall_actual_coarse_offset,
                         uint16_t *last_Phi_E_Selection, uint32_t *last_UQ_UD_EXT, int16_t *last_PHI_E_EXT);

// Encoder initialization functions
void tmc4671_startEncoderInitialization(uint8_t mode, uint8_t *initMode, uint8_t *initState);

// Modes of operation
void tmc4671_switchToMotionMode(uint8_t mode);

// Torque mode
void tmc4671_setTargetTorque_raw(int32_t targetTorque);
int32_t tmc4671_getTargetTorque_raw();
int32_t tmc4671_getActualTorque_raw();

// Torque mode real world unit scaling
void tmc4671_setTargetTorque_mA(uint16_t torqueMeasurementFactor, int32_t targetTorque);
int32_t tmc4671_getTargetTorque_mA(uint16_t torqueMeasurementFactor);
int32_t tmc4671_getActualTorque_mA(uint16_t torqueMeasurementFactor);
int32_t tmc4671_getTargetTorqueFluxSum_mA(uint16_t torqueMeasurementFactor);
int32_t tmc4671_getActualTorqueFluxSum_mA(uint16_t torqueMeasurementFactor);

// Flux
void tmc4671_setTargetFlux_raw(int32_t targetFlux);
int32_t tmc4671_getTargetFlux_raw();
int32_t tmc4671_getActualFlux_raw();

// Flux regulation real world unit scaling
void tmc4671_setTargetFlux_mA(uint16_t torqueMeasurementFactor, int32_t targetFlux);
int32_t tmc4671_getTargetFlux_mA(uint16_t torqueMeasurementFactor);
int32_t tmc4671_getActualFlux_mA(uint16_t torqueMeasurementFactor);

// Torque/flux limit in real world units
void tmc4671_setTorqueFluxLimit_mA(uint16_t torqueMeasurementFactor, int32_t max);
int32_t tmc4671_getTorqueFluxLimit_mA(uint16_t torqueMeasurementFactor);

// Velocity mode
void tmc4671_setTargetVelocity(int32_t targetVelocity);
int32_t tmc4671_getTargetVelocity();
int32_t tmc4671_getActualVelocity();

// Position mode
void tmc4671_setAbsolutTargetPosition(int32_t targetPosition);
void tmc4671_setRelativeTargetPosition(int32_t relativePosition);
int32_t tmc4671_getTargetPosition();
void tmc4671_setActualPosition(int32_t actualPosition);
int32_t tmc4671_getActualPosition();

// Access for register with dependency register
int32_t tmc4671_readFieldWithDependency(RegisterField field, uint8_t dependsReg, uint32_t dependsValue);

#endif /* TMC_IC_TMC4671_H_ */
