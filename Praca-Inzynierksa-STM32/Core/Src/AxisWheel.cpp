#include "AxisWheel.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

AxisWheel::AxisWheel() {}

// Set steering wheel power
void AxisWheel::setPower(uint16_t power){
    this->power = power;

    tmc4671.setTorqueLimit(power);
}

TMC4671_Driver *AxisWheel::getDriver(){
    return &tmc4671;
}

void AxisWheel::setupTMC4671(){
    tmc4671.init();

    tmc4671.setActualPosition(0);
    float angle = 360 * tmc4671.getEncoder()->getPos_f();
    this->resetMetrics(angle);
}

// Set the maximum degrees of rotation
void AxisWheel::setMaxDegrees(uint16_t degrees) {
    degrees &= 0x7FFF;

    if(degrees > 0){
        maxDegreesOfRotation = degrees;
    }
}

// Returns a scaled encoder value between -0x7fff and 0x7fff with a range of degrees and a float between -1 and 1
// Takes an encoder angle in degrees
std::pair<int32_t, float> AxisWheel::scaleEncoderValue(float angle, uint16_t maxDegrees) {
    if(maxDegrees == 0) {
        return std::make_pair<int32_t, float>(0x7FFF, 0.0);
    }

    int32_t value = ((float)0xFFFF / (float)maxDegrees) * angle;
    float value_f = (2.0 / (float)maxDegrees) * angle; // normalize value to the range [-1.0, 1.0]

    // value = std::max<int32_t>(-0x7FFF, std::min<int32_t>(value, 0x7FFF));
    // value_f = std::max(-1.0f, std::min(value_f, 1.0f));

    return {value, value_f};
}

void AxisWheel::sendPositionReport() {
    uint32_t currentTime = HAL_GetTick();
    
    // If we're trying to send too soon after last successful report
    if (currentTime - lastReportTime < USB_REPORT_INTERVAL_MS) {
        // Buffer the current position for next attempt
        pendingHIDInputReport.reportId = REPORT_ID_WHEEL;
        pendingHIDInputReport.axisX = (int16_t)metric.current.position;
        reportPending = true;
        return;
    }

    // If we have a pending report, use that instead of current position
    if (reportPending) {
        HIDreportIn = pendingHIDInputReport;
        reportPending = false;
    } else {
        HIDreportIn.reportId = REPORT_ID_WHEEL;
        HIDreportIn.axisX = (int16_t)metric.current.position;
    }

    // Try to send the report with retries
    USBD_StatusTypeDef status = USBD_FAIL;
    for (uint8_t retry = 0; retry < USB_REPORT_RETRY_COUNT; retry++) {
        status = (USBD_StatusTypeDef) USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, 
                                                                 (uint8_t*)&HIDreportIn, 
                                                                 sizeof(HIDreportIn));
        if (status == USBD_OK) {
            lastReportTime = currentTime;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
            return;
        }
        // Small delay between retries
        if (retry < USB_REPORT_RETRY_COUNT - 1) {
            HAL_Delay(1);
        }
    }

    // If we get here, all retries failed
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
}

void AxisWheel::updateDriverTorque() {
    if(tmc4671.getEncoder() == nullptr) {
        return;
    }
    float angle = 360 * tmc4671.getEncoder()->getPos_f();

    updateMetrics(angle);
    sendPositionReport();

    int32_t totalTorque = 0;
    calculateStaticAxisEffects();
    bool torqueChanged = getAxisTotalTorque(&totalTorque);

    if(torqueChanged && tmc4671.isInitialized()) {
        tmc4671.turn(totalTorque);
    }
}

// Resets the metrics based on position in degrees
void AxisWheel::resetMetrics(float new_position = 0) {
	metric.current = metric_t();
	metric.current.posDegrees = new_position;
	std::tie(metric.current.position, metric.current.position_f) = scaleEncoderValue(new_position, maxDegreesOfRotation);
	metric.previous = metric_t();
}

// Update metrics based on new position
void AxisWheel::updateMetrics(float new_position) {
    // Store old metrics for next metric's computing
    metric.previous = metric.current;

    metric.current.posDegrees = new_position;
    std::tie(metric.current.position, metric.current.position_f) = scaleEncoderValue(new_position, this->maxDegreesOfRotation);

    // compute velocity and accel from raw instant speed normalized
	float currentVelocity = (new_position - metric.previous.posDegrees) * 1000.0; // deg/s
	metric.current.velocity = currentVelocity;
	metric.current.acceleration = (currentVelocity - metric.previous.velocity)* 1000.0; // deg/s^2
}

// Returns force of idle center spring (FFB off) based on position
int32_t AxisWheel::getIdleSpringForce(){
    return clip<int32_t,int32_t>((int32_t)(-metric.current.position * idleCenterSpringFactor), -8000, 8000);
}

// Change the strength of the idle spring
void AxisWheel::changeIdleSpringStrength(uint8_t springStrength) {
    if(springStrength == 0) {
        idleCenterSpring = false;
    }
    else {
        idleCenterSpring = true;
    }
    idleCenterSpringStrength = springStrength;
    idleCenterSpringClip = clip<int32_t,int32_t>((int32_t)springStrength*35, 0, 10000);
    idleCenterSpringFactor = 0.5f + ((float)springStrength * 0.01f);
}

// Change the strength of the FFB effect
void AxisWheel::changeFfbEffectStrength(uint8_t strength) {
    if(strength == 0) {
        ffbOn = false;
    }
    else {
        ffbOn = true;
    }
    ffbEffectStrength = strength;
}

// Set force feedback effects work mode
void AxisWheel::setFfbState(bool ffbState) {
    ffbOn = ffbState;
}

// Calculate soft-lock effect
int16_t AxisWheel::getAndUpdateSoftLock(){
    int8_t clipDirection = 0;
    
    if (metric.current.position > 0x7fff) {
        clipDirection = 1;
    } else if (metric.current.position < -0x7fff) {
        clipDirection = -1;
    }

    if (clipDirection == 0) {
        return 0;
    }

    // Calculate additional torque based on clipping direction and position
    float addtorque = clipDirection * metric.current.posDegrees - (float)this->maxDegreesOfRotation / 2.0; // degrees of rotation counts total range so devide by 2
    addtorque *= (float)softLockStrength * softLockGainFactor; // Apply soft-lock gain for stiffness.
    addtorque *= -clipDirection;

    // Clip the resulting torque value to the allowed range
    return clip<int32_t,int32_t>(addtorque,-0x7fff,0x7fff);
}

void AxisWheel::setSoftLockStrength(uint8_t strength){
    softLockStrength = strength;
}

metric_t AxisWheel::getMetrics() {
    return metric.current;
}

void AxisWheel::calculateStaticAxisEffects(){
    axisEffectTorque = 0;

    if(!ffbOn && idleCenterSpring){
        axisEffectTorque += getIdleSpringForce();
    }

    float velocityFiltered = (metric.current.velocity) * (float)damperIntensity * AXIS_DAMPER_RATIO;
    axisEffectTorque -= clip<float, int32_t>(velocityFiltered, -internalEffectForceClip, internalEffectForceClip);
}

bool AxisWheel::getAxisTotalTorque(int32_t* totalTorque){
    int32_t torque = 0; // In future start with game effect tourqe istead of 0 and multiply it by 80% to have a margin for soft-lock
    // torque += axisEffectTorque;
    torque += getAndUpdateSoftLock();
    torque *= powerTorqueFactor;

    metric.current.torque = torque;
    torque = clip<int32_t, int32_t>(torque, -power, power);
    *totalTorque = torque;

    // if the torque changed return true
    return (metric.current.torque != metric.previous.torque);
}

void AxisWheel::handleHIDCommand(uint8_t cmd, uint8_t* data) {
    switch(cmd) {
        case CMD_INITIALIZE:
            // Initialize doesn't need any data
            pendingCommands.initialize = true;
            HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);
            break;
            
        case CMD_SET_CENTER:
            // Set center doesn't need any data
            pendingCommands.setCenter = true;
            HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);
            break;
            
        case CMD_SET_POWER: {
            // Check if we have any data to read
            if (data == nullptr) {
                return;
            }

            // Power requires 2 bytes of data
            // Read data as a byte array into the value
            uint16_t power = (data[1] << 8) | data[0];
                
            // Validate power value range
            if (power <= 10000) { // Maximum safe power value
                pendingCommands.setPower = true;
                pendingCommands.powerValue = power;
                HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);
            }
            break;
        }
            
        case CMD_SET_SPRING: {
            // Spring requires 1 byte of data
            pendingCommands.setSpring = true;
            pendingCommands.springValue = data[0]; // 0-255 range is valid for spring
            HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);

            break;
        }
            
        case CMD_SET_MAX_ANGLE: {
            // Max angle requires 2 bytes of data
            uint16_t maxAngle = (data[1] << 8) | data[0];
            
            // Validate max angle value range
            if (maxAngle <= 1440) { // Maximum safe angle value
                pendingCommands.setMaxAngle = true;
                pendingCommands.maxAngleValue = maxAngle;
                HAL_GPIO_TogglePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin);
            }
            break;
        }
    }
}

void AxisWheel::processCommands() {
    // Process any pending commands
    if (pendingCommands.initialize) {
        setupTMC4671();
        pendingCommands.initialize = false;
    }
    
    if (pendingCommands.setCenter) {
        this->tmc4671.getEncoder()->setActualPosition(0);
        pendingCommands.setCenter = false;
    }
    
    if (pendingCommands.setPower) {
        setPower(pendingCommands.powerValue);
        pendingCommands.setPower = false;
        pendingCommands.powerValue = 0;
    }
    
    if (pendingCommands.setSpring) {
        changeIdleSpringStrength(pendingCommands.springValue);
        pendingCommands.setSpring = false;
        pendingCommands.springValue = 0;
    }
    
    if (pendingCommands.setMaxAngle) {
        setMaxDegrees(pendingCommands.maxAngleValue);
        pendingCommands.setMaxAngle = false;
        pendingCommands.maxAngleValue = 0;
    }
}
