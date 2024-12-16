/*
 * util_functions.h
 *
 *  Created on: Dec 10, 2024
 *      Author: dawid
 */

#ifndef INC_AXISWHEEL_FUNCTIONS_H_
#define INC_AXISWHEEL_FUNCTIONS_H_

#include <inttypes.h> 
#include <utility>
#include <tuple>

#include "util_functions.h"
#include "TMC4671_controller.h"
#include <memory>

#define INTERNAL_SCALER_DAMPER 40
#define INTERNAL_SCALER_FRICTION 45
#define INTERNAL_SCALER_INERTIA 4

#define INTERNAL_AXIS_DAMPER_SCALER 0.7
#define INTERNAL_AXIS_FRICTION_SCALER 0.7
#define INTERNAL_AXIS_INERTIA_SCALER 0.7

struct metric_t {
	float acceleration  = 0;   // in deg/s²
	float velocity      = 0;   // in deg/s
	int32_t position    = 0;   // scaled position as 16b int -0x7fff to 0x7fff
	float position_f    = 0;   // scaled position as float. -1 to 1 range
	float posDegrees    = 0;   // Position in degrees. Not scaled to selected range
	int32_t torque      = 0;   // total of effect + endstop torque
};

struct axis_metric_t {
	metric_t current;
	metric_t previous;
};

class AxisWheel
{
public:
    AxisWheel();

    void setPower(uint16_t power);
    TMC4671_Driver* getDriver();
    void setupTMC4671();
    void setMaxDegrees(uint16_t degrees);
    std::pair<int32_t,float> scaleEncoderValue( float angle, uint16_t maxDegrees);
    void updateDriverTorque();

    void resetMetrics(float new_position);
    void updateMetrics(float new_position);
    metric_t getMetrics();

    void changeIdleSpringStrength(uint8_t springStrength);
    int32_t getIdleSpringForce();
    void changeFfbEffectStrength(uint8_t strength);
    void setFfbState(bool ffbState);

    int16_t getAndUpdateSoftLock();
    void setSoftLockStrength(uint8_t strength);

    void calculateStaticAxisEffects();
    void getAxisTotalTorque(int32_t* torque);


private:
    // Axis damper is lower than default scale of HID Damper
	const float AXIS_DAMPER_RATIO = INTERNAL_SCALER_DAMPER * INTERNAL_AXIS_DAMPER_SCALER / 255.0;
	const float AXIS_INERTIA_RATIO = INTERNAL_SCALER_INERTIA * INTERNAL_AXIS_INERTIA_SCALER / 255.0;

    std::unique_ptr<TMC4671_Driver> tmc4671 = std::make_unique<TMC4671_Driver>();
    axis_metric_t metric;
    int32_t effectTorque = 0;
	int32_t axisEffectTorque = 0;
    const int32_t internalEffectForceClip = 20000;
    uint8_t damperIntensity = 30;
    uint16_t power = 7000;
    float powerTorqueFactor = 0.6;
    uint16_t maxDegreesOfRotation = 900;

    const float ffbEffectScaler = 1.0;
    uint8_t ffbEffectStrength = 127;
    bool ffbOn = false;

    bool idleCenterSpring = true;
    uint8_t idleCenterSpringStrength = 127;
    int16_t idleCenterSpringClip = 4445;
    float idleCenterSpringFactor = 1.0;

    uint8_t softLockStrength = 127;
    const float softLockGainFactor = 25.0;
};

#endif // INC_AXISWHEEL_FUNCTIONS_H_
