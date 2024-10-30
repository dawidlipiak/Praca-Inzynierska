/*
 * TMC4671_controller.c
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */

#include <TMC4671_controller.hpp>

TMC4671::TMC4671()
{}

TMC4671::~TMC4671()
{}

void TMC4671::init(void)
{
    // Inicjalizacja sterownika TMC4671
}

void TMC4671::deInit(void)
{
    this->enableDriver(DRIVER_DISABLE);
}

uint32_t TMC4671::rotate(int32_t velocity)
{
	// switch to velocity motion mode
	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_VELOCITY);

	// set target velocity for ramp generator
	rampGenerator.targetVelocity = velocity;

	// remember switched motion mode
	actualMotionMode = TMC4671_MOTION_MODE_VELOCITY;

	return TMC_ERROR_NONE;
}

uint32_t TMC4671::moveTo(int32_t position)
{
    // Funkcja do przesunięcia silnika do określonej pozycji
    return 0;
}

uint32_t TMC4671::moveBy(int32_t *ticks)
{
    // Funkcja do przesunięcia o określoną liczbę kroków
    return 0;
}

uint8_t TMC4671::positionReached(int32_t targetPosition, int32_t actualPosition, int32_t actualVelocity, int32_t maxPosDiff, int32_t maxVel)
{
    // Sprawdzenie, czy pozycja została osiągnięta
    return 0;
}

void TMC4671::enableDriver(DriverState state)
{
	if(state == DRIVER_DISABLE){
		this->driverState = DRIVER_DISABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	}
	else {
		this->driverState = DRIVER_ENABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
	}
}

void TMC4671::periodicJob(uint32_t actualSystick)
{
    // Funkcja okresowa do aktualizacji stanu sterownika
}

int32_t TMC4671::tmc_filterPT1(int64_t *akku, int32_t newValue, int32_t lastValue, uint8_t actualFilter, uint8_t maxFilter)
{
    // Funkcja realizująca filtr pierwszego rzędu PT1
    return 0;
}
