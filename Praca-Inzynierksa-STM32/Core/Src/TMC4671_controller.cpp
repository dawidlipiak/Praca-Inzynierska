/*
 * TMC4671_controller.c
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */


#include <TMC4671_controller.h>

TMC4671_Driver tmc4671;

void TMC4671_Driver::setDriverState(DriverState state) {
	if (state == DRIVER_DISABLE) {
		this->driverState = DRIVER_DISABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	} else {
		this->driverState = DRIVER_ENABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
	}
}

void TMC4671_Driver::setMotionMode(MotionMode mode){
//	if(!force){
//		nextMotionMode = mode;
//		return;
//	}
//	if(mode != curMotionMode){
//		lastMotionMode = curMotionMode;
//	}
	this->curr_motionMode = mode;
	tmc4671_fieldWrite(TMC4671_MODE_MOTION_FIELD, (uint8_t) mode);
}

MotionMode TMC4671_Driver::getMotionMode(){
	this->curr_motionMode = (MotionMode) tmc4671_fieldRead(TMC4671_MODE_MOTION_FIELD);
	return this->curr_motionMode;
}

PhiE TMC4671_Driver::getPhiEType(){
	this->phiEType = (PhiE) (tmc4671_fieldRead(TMC4671_PHI_E_SELECTION_FIELD) & 0x07);
	return this->phiEType;
}

int16_t TMC4671_Driver::getPhiE(){
	return (int16_t)tmc4671_readRegister(TMC4671_PHI_E);
}

/**
 * Reads phiE directly from the encoder selection instead of the current phiE selection
 */
int16_t TMC4671_Driver::getPhiE_Enc(){
	if(this->encoderType == EncoderType::abn){
		return (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD);
	}
	else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
		return (int16_t)tmc4671_fieldRead(TMC4671_AENC_DECODER_PHI_E_FIELD);
	}
	else if(this->encoderType == EncoderType::hall){
		return (int16_t)tmc4671_fieldRead(TMC4671_HALL_PHI_E_FIELD);
	}
//	else if(usingExternalEncoder()){
//		return getPhiEfromExternalEncoder();
//	}
	else{
		return getPhiE();
	}
}

void TMC4671_Driver::setMotorTypeAndPoles(MotorType motor, uint16_t poles){
	if(motor == MotorType::DC){
		poles = 1;
	}
	this->motorType = motor;
	this->encoder.pole_pairs = poles;

	tmc4671_fieldWrite(TMC4671_N_POLE_PAIRS_FIELD, poles);
	tmc4671_fieldWrite(TMC4671_MOTOR_TYPE_FIELD, (uint8_t)motor);

}

void TMC4671_Driver::setPhiEType(PhiE phiEType){
	if(phiEType == PhiE::extEncoder){
		phiEType = PhiE::ext;
	}
	this->phiEType = phiEType;

	tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, (uint8_t)phiEType && 0xFF);
}

void TMC4671_Driver::setHallConfig(HallConfig* hallConfig_p){
	memcpy(&this->hallConfig, hallConfig_p, sizeof(this->hallConfig));

	tmc4671_fieldWrite(TMC4671_HALL_POLARITY_FIELD, hallConfig_p->polarity);
	tmc4671_fieldWrite(TMC4671_HALL_SYNCHRONOUS_PWM_SAMPLING_FIELD, hallConfig_p->pwmSamplingFilter);
	tmc4671_fieldWrite(TMC4671_HALL_INTERPOLATION_FIELD, hallConfig_p->interpolation);
	tmc4671_fieldWrite(TMC4671_HALL_DIRECTION_FIELD, hallConfig_p->direction);
	tmc4671_fieldWrite(TMC4671_HALL_BLANK_FIELD, hallConfig_p->blank);

	tmc4671_fieldWrite(TMC4671_HALL_POSITION_000_FIELD, hallConfig_p->pos0);
	tmc4671_fieldWrite(TMC4671_HALL_POSITION_060_FIELD, hallConfig_p->pos60);
	tmc4671_fieldWrite(TMC4671_HALL_POSITION_120_FIELD, hallConfig_p->pos120);
	tmc4671_fieldWrite(TMC4671_HALL_POSITION_180_FIELD, hallConfig_p->pos180);
	tmc4671_fieldWrite(TMC4671_HALL_POSITION_240_FIELD, hallConfig_p->pos240);
	tmc4671_fieldWrite(TMC4671_HALL_POSITION_300_FIELD, hallConfig_p->pos300);

	tmc4671_fieldWrite(TMC4671_HALL_PHI_M_OFFSET_FIELD, hallConfig_p->phiMoffset);
	tmc4671_fieldWrite(TMC4671_HALL_PHI_E_OFFSET_FIELD, hallConfig_p->phiEoffset);
	tmc4671_fieldWrite(TMC4671_HALL_DPHI_MAX_FIELD, hallConfig_p->dPhiMax);
}

void TMC4671_Driver::setPWM(PwmMode pwmMode){
	this->pwmMode = pwmMode;
	tmc4671_fieldWrite(TMC4671_PWM_CHOP_FIELD, (uint8_t)pwmMode);
}

void TMC4671_Driver::setPWM(PwmMode pwmMode,uint16_t maxcnt,uint8_t bbmL,uint8_t bbmH){
	maxcnt = clip(maxcnt, 255, 4095);
	this->pwmMode = pwmMode;
	this->pwmCnt = maxcnt;
	this->bbmL = bbmL;
	this->bbmH = bbmH;

	tmc4671_fieldWrite(TMC4671_PWM_MAXCNT_FIELD, maxcnt);
	tmc4671_fieldWrite(TMC4671_PWM_CHOP_FIELD, (uint8_t)pwmMode);
	tmc4671_fieldWrite(TMC4671_PWM_BBM_L_FIELD, bbmL);
	tmc4671_fieldWrite(TMC4671_PWM_BBM_H_FIELD, bbmH);

	tmc4671_writeRegister(TMC4671_PWM_POLARITIES, 0x00);
}

void TMC4671_Driver::setAdcOffset(AdcConfig* adcConfig_p){
	this->adcConfig.adc_I0_offset = adcConfig_p->adc_I0_offset;
	this->adcConfig.adc_I1_offset = adcConfig_p->adc_I1_offset;

	tmc4671_fieldWrite(TMC4671_ADC_I0_OFFSET_FIELD, adcConfig_p->adc_I0_offset);
	tmc4671_fieldWrite(TMC4671_ADC_I1_OFFSET_FIELD, adcConfig_p->adc_I1_offset);
}

void TMC4671_Driver::setAdcScale(AdcConfig* adcConfig_p){
	this->adcConfig.adc_I0_scale = adcConfig_p->adc_I0_scale;
	this->adcConfig.adc_I1_scale = adcConfig_p->adc_I1_scale;

	tmc4671_fieldWrite(TMC4671_ADC_I0_SCALE_FIELD, adcConfig_p->adc_I0_scale);
	tmc4671_fieldWrite(TMC4671_ADC_I1_SCALE_FIELD, adcConfig_p->adc_I1_scale);
}

void TMC4671_Driver::initAdc(AdcConfig* adcConfig_p){
	this->adcConfig.mclkA = adcConfig_p->mclkA;
	this->adcConfig.mclkB = adcConfig_p->mclkB;
	this->adcConfig.mdecA = adcConfig_p->mdecA;
	this->adcConfig.mdecB = adcConfig_p->mdecB;

	tmc4671_fieldWrite(TMC4671_DSADC_MDEC_A_FIELD, adcConfig_p->mdecA);
	tmc4671_fieldWrite(TMC4671_DSADC_MDEC_B_FIELD, adcConfig_p->mdecB);
	tmc4671_fieldWrite(TMC4671_DSADC_MCLK_A_FIELD, adcConfig_p->mclkA);
	tmc4671_fieldWrite(TMC4671_DSADC_MCLK_B_FIELD, adcConfig_p->mclkB);
	tmc4671_fieldWrite(TMC4671_SEL_NCLK_MCLK_I_A_FIELD, adcConfig_p->mclkA == 0 ? 0 : 1);
	tmc4671_fieldWrite(TMC4671_SEL_NCLK_MCLK_I_B_FIELD, adcConfig_p->mclkB == 0 ? 0 : 1);

	tmc4671_fieldWrite(TMC4671_ADC_I0_SELECT_FIELD, 0x0);   // 0: ADCSD_I0_RAW (sigma delta ADC)
	tmc4671_fieldWrite(TMC4671_ADC_I1_SELECT_FIELD, 0x01);  // 1: ADCSD_I1_RAW (sigma delta ADC)
	tmc4671_fieldWrite(TMC4671_ADC_I_UX_SELECT_FIELD,0x00); // 0: UX = ADC_I0 (default)
	tmc4671_fieldWrite(TMC4671_ADC_I_V_SELECT_FIELD, 0x02); // 2: V = ADC_I2
	tmc4671_fieldWrite(TMC4671_ADC_I_WY_SELECT_FIELD, 0x01);// 1: WY = ADC_I1

	setAdcOffset(adcConfig_p);
	setAdcScale(adcConfig_p);
}

void TMC4671_Driver::setPids(PIDConfig* pidConfig_p){
	memcpy(&this->pidConfig, pidConfig_p, sizeof(this->pidConfig));

	tmc4671_fieldWrite(TMC4671_PID_FLUX_I_FIELD, pidConfig_p->fluxI);
	tmc4671_fieldWrite(TMC4671_PID_FLUX_P_FIELD, pidConfig_p->fluxP);
	tmc4671_fieldWrite(TMC4671_PID_TORQUE_I_FIELD, pidConfig_p->torqueI);
	tmc4671_fieldWrite(TMC4671_PID_TORQUE_P_FIELD, pidConfig_p->fluxP);
	tmc4671_fieldWrite(TMC4671_PID_VELOCITY_I_FIELD, pidConfig_p->velocityI);
	tmc4671_fieldWrite(TMC4671_PID_VELOCITY_P_FIELD, pidConfig_p->velocityP);
	tmc4671_fieldWrite(TMC4671_PID_POSITION_I_FIELD, pidConfig_p->positionI);
	tmc4671_fieldWrite(TMC4671_PID_POSITION_P_FIELD, pidConfig_p->positionP);
	tmc4671_fieldWrite(TMC4671_MODE_PID_TYPE_FIELD, pidConfig_p->sequentialPI);
}

void TMC4671_Driver::setPidPrecision(PidPrecision* setting_p){

	memcpy(&this->pidPrecision, setting_p, sizeof(this->pidPrecision));
	uint16_t dat = setting_p->current_I;
	dat |= setting_p->current_P << 1;
	dat |= setting_p->velocity_I << 2;
	dat |= setting_p->velocity_P << 3;
	dat |= setting_p->position_I << 4;
	dat |= setting_p->position_P << 5;
	tmc4671_writeRegister(TMC4671_CONFIG_ADDR, 62); // set config register address
	tmc4671_writeRegister(TMC4671_CONFIG_DATA, dat);
}

/**
 *  Sets the raw brake resistor limits.
 *  Centered at 0x7fff
 *  Set both 0 to deactivate
 */
void TMC4671_Driver::setAdcBrakeLimits(uint16_t low,uint16_t high){
	this->brakeLimLow = low;
	this->brakeLimHigh = high;

	tmc4671_fieldWrite(TMC4671_ADC_VM_LIMIT_LOW_FIELD, low);
	tmc4671_fieldWrite(TMC4671_ADC_VM_LIMIT_LOW_FIELD, high);
}

// Changes actual multi turn position for positioning
void TMC4671_Driver::setActualPosition(int32_t pos){
	tmc4671_fieldWrite(TMC4671_PID_POSITION_ACTUAL_FIELD, (uint32_t)pos);
}

// Returns actual multi turn position from tmc
int32_t TMC4671_Driver::getActualPosition(){
	return (int32_t)tmc4671_fieldRead(TMC4671_PID_POSITION_ACTUAL_FIELD);
}

int32_t TMC4671_Driver::getAbsolutePosition(){
	int16_t pos;

	if(this->encoderType == EncoderType::abn){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_M_FIELD);
	}
	else if(this->encoderType == EncoderType::hall){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_HALL_PHI_M_FIELD);
	}
	else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_AENC_DECODER_PHI_M_FIELD);
	}
	else{
//		pos = getPos(); // read phiM
	}

	return pos;
}

void TMC4671_Driver::setFluxTorque(int16_t flux, int16_t torque){
//	if(curr_motionMode != MotionMode::torque){
		setMotionMode(MotionMode::torque);
//	}

//	tmc4671_fieldWrite(TMC4671_PID_FLUX_TARGET_FIELD, flux);
//	tmc4671_fieldWrite(TMC4671_PID_TORQUE_TARGET_FIELD, torque);
	tmc4671_writeRegister(0x64, (flux & 0xffff) | (torque << 16));
	tmc4671_writeRegister(0x64, (flux & 0xffff) | (torque << 16));
}

void TMC4671_Driver::setStatusFlags(StatusFlags flag){
	tmc4671_writeRegister(TMC4671_STATUS_MASK, flag.asInt);
}

void TMC4671_Driver::setStatusMask(StatusFlags mask){
	tmc4671_writeRegister(TMC4671_STATUS_MASK, mask.asInt);
}

void TMC4671_Driver::setOpenLoopSpeedAccel(int32_t speed,uint32_t accel){
	tmc4671_writeRegister(TMC4671_OPENLOOP_VELOCITY_TARGET, speed);
	tmc4671_writeRegister(TMC4671_OPENLOOP_ACCELERATION, accel);
}

void TMC4671_Driver::runOpenLoop(uint16_t ud,uint16_t uq,int32_t speed,int32_t accel,bool torqueMode){
	if(this->motorType == MotorType::DC){
		uq = ud+uq; // dc motor has no flux. add to torque
	}

	if(torqueMode){
		if(this->motorType == MotorType::DC){
			uq = ud+uq; // dc motor has no flux. add to torque
		}
		setFluxTorque(ud, uq);
	}

	else{
		setMotionMode(MotionMode::uqudext);
		setUdUq(ud,uq);
	}

	int16_t oldPhiE = getPhiE();
	setPhiEType(PhiE::openloop);
	tmc4671_writeRegister(TMC4671_OPENLOOP_PHI,oldPhiE); // Start running at last phiE value

	setOpenLoopSpeedAccel(speed, accel);
}

void TMC4671_Driver::setUdUq(int16_t ud,int16_t uq){
	tmc4671_fieldWrite(TMC4671_UD_EXT_FIELD, ud);
	tmc4671_fieldWrite(TMC4671_UQ_EXT_FIELD, uq);
}

void TMC4671_Driver::setPhiE_ext(int16_t phiE){
	tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, (uint32_t)phiE);
}

/**
 * Calibrates the ADC by disabling the power stage and sampling a mean value. Takes time!
 */
bool TMC4671_Driver::calibrateAdcOffset(uint16_t time){

	uint16_t measuretime_idle = time;
	uint32_t measurements_idle = 0;
	uint64_t totalA=0;
	uint64_t totalB=0;

	tmc4671_writeRegister(TMC4671_ADC_RAW_ADDR, 0); // Read raw adc
	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();
	setMotionMode(MotionMode::stopped);
	HAL_Delay(100); // Wait a bit before sampling
	uint16_t lastrawA = this->adcConfig.adc_I0_offset, lastrawB = this->adcConfig.adc_I1_offset;

	//pulseClipLed(); // Turn on led
	// Disable drivers and measure many samples of zero current
	//enablePin.reset();
	uint32_t tick = HAL_GetTick();
	while(HAL_GetTick() - tick < measuretime_idle){ // Measure idle
		tmc4671_writeRegister(TMC4671_ADC_RAW_ADDR, 0); // Read raw adc
		uint32_t adcraw = tmc4671_readRegister(TMC4671_ADC_RAW_DATA);
		uint16_t rawA = adcraw & 0xffff;
		uint16_t rawB = (adcraw >> 16) & 0xffff;

		// Signflip filter for SPI bug
		if(abs(lastrawA-rawA) < 10000 && abs(lastrawB-rawB) < 10000){
			totalA += rawA;
			totalB += rawB;
			measurements_idle++;
			lastrawA = rawA;
			lastrawB = rawB;
		}
//		uint32_t lastMicros = micros();
//		while(micros()-lastMicros < 100){} // Wait 100µs at least
	}
	//enablePin.set();
	int32_t offsetAidle = totalA / (measurements_idle);
	int32_t offsetBidle = totalB / (measurements_idle);

	// Check if offsets are in a valid range
	if(totalA < 100 || totalB < 100 || ((abs(offsetAidle - 0x7fff) > TMC_ADCOFFSETFAIL) || (abs(offsetBidle - 0x7fff) > TMC_ADCOFFSETFAIL)) ){
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//		ErrorHandler::addError(Error(ErrorCode::adcCalibrationError,ErrorType::critical,"TMC ADC offset calibration failed."));
//		blinkErrLed(100, 0); // Blink forever
//		setPwm(TMC_PwmMode::off); //Disable pwm
//		this->changeState(TMC_ControlState::HardError);
		this->adcConfig.adcCalibrated = false;
		return false; // An adc or shunt amp is likely broken. do not proceed.
	}
	this->adcConfig.adc_I0_offset = offsetAidle;
	this->adcConfig.adc_I1_offset = offsetBidle;
	setAdcOffset(&this->adcConfig);
	// ADC Offsets should now be close to perfect

	setPhiEType(lastphie);
	setMotionMode(lastmode);
	this->adcConfig.adcCalibrated = true;
	return true;
}

/**
 * Moves the rotor and estimates polarity and direction of the encoder
 * Polarity is found by measuring the n pulse.
 * If polarity was found to be reversed during the test direction will be reversed again to account for that
 */
void TMC4671_Driver::estimateABNparams(){
	int32_t pos = getActualPosition();
	setActualPosition(0);
	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();
	tmc4671_fieldWrite(TMC4671_ABN_DIRECTION_FIELD, 0); // Set direction positive (0)

	setPhiE_ext(0); // Electrical angle phi_e_ext for external writing into this register
	setPhiEType(PhiE::ext);
	setFluxTorque(0, 0);
	setMotionMode(MotionMode::torque);

	for(int16_t flux = 0; flux <= initPower; flux+=20){
		setFluxTorque(flux, 0);
		HAL_Delay(5);
	}

	int16_t phiE_abn = tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD);
	int16_t phiE_abn_old = 0;
	int16_t rcount = 0, c = 0; // Count how often direction was in reverse
	uint16_t highcount = 0; // Count high state of n pulse for polarity estimation

	// Rotate a bit
	for(int16_t p = 0;p<0x0fff;p+=0x2f){
		setPhiE_ext(p);
		HAL_Delay(10);
		c++;
		phiE_abn_old = phiE_abn;
		phiE_abn = tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD);

		// Count how often the new position was lower than the previous indicating a reversed encoder or motor direction
		if(phiE_abn < phiE_abn_old){
			rcount++;
		}

		if(tmc4671_fieldRead(TMC4671_N_OF_ABN_RAW_FIELD)){
			highcount++;
		}
	}
	setActualPosition(pos + getActualPosition());

	setFluxTorque(0, 0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);

	bool npol = highcount > c/2;
	encoder.rdir = rcount > c/2;

	if(npol != encoder.npol){ // Invert dir if polarity was reversed TODO correct? likely wrong at the moment
		encoder.rdir = !encoder.rdir;
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	}


	encoder.apol = npol;
	encoder.bpol = npol;
	encoder.npol = npol;

//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//	HAL_Delay(200);
//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
}

void TMC4671_Driver::zeroAbnUsingPhiM(bool offsetPhiE = false){
	int32_t npos = tmc4671_readRegister(TMC4671_ABN_DECODER_COUNT_N); // raw encoder counts at index hit
	int32_t npos_M = (npos * 0xffff) / encoder.cpr; // Scaled encoder angle at index
	encoder.phiMoffset = -npos_M;

	if(offsetPhiE){
		encoder.phiEoffset += npos_M * encoder.pole_pairs;
	}else{
		tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_M_OFFSET_FIELD, encoder.phiMoffset);
	}

	setActualPosition(getAbsolutePosition()); // Set position to absolute position = ~zero
}



/**
 * Aligns ABN encoders by forcing an angle with high current and calculating the offset
 * Will start at the current phiE to minimize any extra movements (useful if motor was turned in openloop mode before already)
 * @param power Maximum current reached during flux ramp
 */
void TMC4671_Driver::powerInitEncoder(int16_t power){
	// This aligning technique is only for stepper and bldc motors
	if(motorType != MotorType::STEPPER && motorType != MotorType::BLDC){
		return;
	}

	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();
	setFluxTorque(0, 0);

	RegisterField phiEoffsetReg = TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD;

	if(this->encoderType == EncoderType::abn){
		phiEoffsetReg = TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD;

		if(!encoder.indexHitFlag){
			zeroAbnUsingPhiM();
		}
	}
	else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
		tmc4671_writeRegister(TMC4671_AENC_DECODER_COUNT, 0); //Zero encoder
		tmc4671_writeRegister(TMC4671_AENC_DECODER_POSITION, 0); //Zero encoder
		phiEoffsetReg = TMC4671_AENC_DECODER_PHI_E_OFFSET_FIELD;
	}
//	else if (usingExternalEncoder()){
//		externalEncoderPhieOffset = 0;
//	}
//	else{
//		return; // Not relevant
//	}

	int16_t phiEpos = getPhiE(); // starts at current encoder position
	tmc4671_fieldWrite(phiEoffsetReg, 0); // Set phiE offset to zero
	setPhiE_ext(phiEpos);
	setPhiEType(PhiE::ext);

	// Ramp up flux
	for(int16_t flux = 0; flux <= power; flux+=10){
		setFluxTorque(flux, 0);
		HAL_Delay(3);
	}

	int16_t phiE_enc = getPhiE_Enc();
	HAL_Delay(50);
	int16_t phiE_abn_old = 0;
	int16_t c = 0;
	uint16_t still = 0;

//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	while(still < 30 && c++ < 1000){
		// Wait for motor to stop moving
		if(abs(phiE_enc - phiE_abn_old) < 100){
			still++;
		}
		else{
			still = 0;
		}
		phiE_abn_old = phiE_enc;
		phiE_enc = getPhiE_Enc();
		HAL_Delay(10);
	}
	setFluxTorque(0, 0);
//	HAL_Delay(300);
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);

	//Write offset
	int16_t phiEoffset =  phiEpos-phiE_enc;

	if(phiEoffset == 0){ // 0 invalid
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
//		HAL_Delay(5000);
		phiEoffset = 1;
	}

//	if (usingExternalEncoder()){
//		externalEncoderPhieOffset = phiEoffset;
//	}
//	else{
	tmc4671_fieldWrite(phiEoffsetReg, phiEoffset);
//	}

	if(this->encoderType == EncoderType::abn){
		encoder.phiEoffset = phiEoffset;
	}
	else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
		encoder.phiEoffset = phiEoffset;
	}


	setPhiE_ext(0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);
}

/**
 * Steps the motor a few times to check if the encoder follows correctly
 */
bool TMC4671_Driver::checkEncoder(){
	if(this->motorType != MotorType::STEPPER && this->motorType != MotorType::BLDC){
		// If not stepper or bldc return
		return true;
	}

	const uint16_t maxcount = 80; // Allowed reversals
	const uint16_t maxfail = 10; // Allowed fails

	// Start angle offsets all angles later so there is no jump if angle is already properly aligned
	const int16_t startAngle = getPhiE_Enc();
	const int16_t targetAngle = 0x3FFF;

	bool result = true;
	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();

//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//	HAL_Delay(200);
//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);

	setFluxTorque(0, 0);
	setPhiEType(PhiE::ext);
	setPhiE_ext(startAngle);

	// Ramp up flux
	for(int16_t flux = 0; flux <= 2*this->initPower/3; flux+=20){
		setFluxTorque(flux, 0);
		HAL_Delay(2);
	}

	//Forward
	int16_t phiE_enc = 0;
	uint16_t failcount = 0;
	int16_t revCount = 0;
	for(int16_t angle = 0; angle<targetAngle; angle+=0x00ff){
		uint16_t c = 0;
		setPhiE_ext(angle+startAngle);
		HAL_Delay(5);

		phiE_enc = getPhiE_Enc() - startAngle;
		int16_t err = abs(phiE_enc - angle);
		int16_t nErr = abs(phiE_enc + angle);

		// Wait more until encoder settles a bit
		while(err > 2500 && nErr > 2500 && c++ < maxcount){
			phiE_enc = getPhiE_Enc() - startAngle;
			err = abs(phiE_enc - angle);
			nErr = abs(angle - phiE_enc);
			HAL_Delay(10);
		}

		if(err > nErr){
			revCount++;
		}

		if(c >= maxcount){
			failcount++;
			if(failcount > maxfail){
				result = false;
				HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
				HAL_Delay(300);
				HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
				break;
			}
		}
	}

	/* If we are still at the start angle the encoder did not move at all.
	 * Possible issues:
	 * Encoder connection wrong
	 * Wrong encoder selection
	 * No motor movement
	 * No encoder power
	 */
	if(startAngle == getPhiE_Enc()){
		// TODO: error handler
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
		result = false;
	}

	// Backward

	if(result){ // Only if not already failed
		for(int16_t angle = targetAngle;angle>0;angle -= 0x00ff){
			uint16_t c = 0;
			setPhiE_ext(angle+startAngle);
			HAL_Delay(5);

			phiE_enc = getPhiE_Enc() - startAngle;
			int16_t err = abs(phiE_enc - angle);
			int16_t nErr = abs(phiE_enc + angle);

			// Wait more
			while(err > 2500 && nErr > 2500 && c++ < maxcount){
				phiE_enc = getPhiE_Enc() - startAngle;
				err = abs(phiE_enc - angle);
				nErr = abs(angle - phiE_enc);
				HAL_Delay(10);
			}

			if(err > nErr){
				revCount++;
			}

			if(c >= maxcount){
				failcount++;
				if(failcount > maxfail){
					result = false;
					HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
					HAL_Delay(150);
					HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
					break;
				}
			}
		}
	}

	// TODO check if we want that
	if(revCount > maxcount){ // Encoder seems reversed
		// reverse encoder
		if(this->encoderType == EncoderType::abn){
			this->encoder.rdir = !this->encoder.rdir;
//			this->encoder.apol = !this->encoder.apol;
//			this->encoder.bpol = !this->encoder.bpol;
//			this->encoder.npol = !this->encoder.npol;
//			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
//			HAL_Delay(500);
//			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
			tmc4671_fieldWrite(TMC4671_ABN_DIRECTION_FIELD, this->encoder.rdir);
//			tmc4671_fieldWrite(TMC4671_ABN_APOL_FIELD, this->encoder.apol);
//			tmc4671_fieldWrite(TMC4671_ABN_BPOL_FIELD, this->encoder.bpol);
//			tmc4671_fieldWrite(TMC4671_ABN_NPOL_FIELD, this->encoder.npol);
//			result = false;
//			setupAbnEncoder(&encoder);
		}
		else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
			this->encoder.rdir = !this->encoder.rdir;
//			setup_AENC(aencconf);
		}
		else if(this->encoderType == EncoderType::ext){
//			this->encoderReversed = !this->encoderReversed;
		}
//		ErrorHandler::addError(Error(ErrorCode::encoderReversed,ErrorType::warning,"Encoder direction reversed during check"));
	}

	setFluxTorque(0, 0);
	setPhiE_ext(0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);

	if(result){
		encoder.isAligned = true;
	}

	return result;
}

void TMC4671_Driver::setupAbnEncoder(ABNencoder* abnEncoder_p){
	this->statusMask.flags.AENC_N = 0;
	this->statusMask.flags.ENC_N = 0;
	setStatusMask(statusMask);

	memcpy(&this->encoder, abnEncoder_p, sizeof(this->encoder));

	tmc4671_fieldWrite(TMC4671_ABN_APOL_FIELD, abnEncoder_p->apol);
	tmc4671_fieldWrite(TMC4671_ABN_BPOL_FIELD, abnEncoder_p->bpol);
	tmc4671_fieldWrite(TMC4671_ABN_NPOL_FIELD, abnEncoder_p->npol);
	tmc4671_fieldWrite(TMC4671_USE_ABN_AS_N_FIELD, abnEncoder_p->ab_as_n);
	tmc4671_fieldWrite(TMC4671_ABN_CLN_FIELD, abnEncoder_p->latch_on_N);
	tmc4671_fieldWrite(TMC4671_ABN_DIRECTION_FIELD, abnEncoder_p->rdir);

	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PPR_FIELD, abnEncoder_p->cpr);

	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, abnEncoder_p->phiEoffset);
	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_M_OFFSET_FIELD, abnEncoder_p->phiMoffset);

	// Set mechanical angle
	this->encoder.posSelection = PosAndVelSelection::PhiM_abn;
	this->encoder.velSelection = PosAndVelSelection::PhiM_abn;
	tmc4671_fieldWrite(TMC4671_POSITION_SELECTION_FIELD, (uint8_t)this->encoder.posSelection);
	tmc4671_fieldWrite(TMC4671_VELOCITY_SELECTION_FIELD, (uint8_t)this->encoder.velSelection);
	tmc4671_fieldWrite(TMC4671_VELOCITY_METER_SELECTION_FIELD, 0); // 0: default velocity meter (fixed frequency sampling)


	estimateABNparams();

	// find index
//	if(encoderType == EncoderType::abn && encoder.useIndex && !encoder.indexHitFlag){ // TODO changing direction might invalidate phiE offset because of index pulse width
//		findEncoderIndex(encoder.posOffsetFromIndex < 0 ? 10 : -10,this->initPower/2,true,true); // Go to index and zero encoder
//		setPhiEType(PhiE::openloop); // Openloop used in last step. Use for aligning too
//		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//		HAL_Delay(400);
//		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
//	}
//	else{
		setPhiE_ext(tmc4671_readRegister(TMC4671_PHI_E));
		setPhiEType(PhiE::ext);
//		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//		HAL_Delay(400);
//		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
//	}

	// Align encoder
	powerInitEncoder(this->initPower);

	uint8_t enc_retry = 0;
	while(!encoder.isAligned && enc_retry < 3){
		checkEncoder();
		enc_retry++;
	}
	if(!encoder.isAligned) {
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		setDriverState(DRIVER_DISABLE);
		return;
	}

//	if(this->encoderType == EncoderType::abn && encoder.useIndex && encoder.indexHitFlag){
//		setActualPosition(getAbsolutePosition() - encoder.posOffsetFromIndex); // Load stored position
//		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//		HAL_Delay(400);
//		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
//	}

	if(this->encoderType == EncoderType::abn){
		setPhiEType(PhiE::abn);
	}
	else if(this->encoderType == EncoderType::sincos || this->encoderType == EncoderType::uvw){
		setPhiEType(PhiE::aenc);
	}
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1000);
//	else if(usingExternalEncoder()){
//		setPhiEType(PhiE::extEncoder);
//	}

}

/**
 * Applies a biquad filter to the flux target
 * Set nullptr to disable
 */
void TMC4671_Driver::setBiquadFlux(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.flux = filter;
	tmc4671_writeRegister(0x4E, 25);
	tmc4671_writeRegister(0x4D, bq.a1);
	tmc4671_writeRegister(0x4E, 26);
	tmc4671_writeRegister(0x4D, bq.a2);
	tmc4671_writeRegister(0x4E, 28);
	tmc4671_writeRegister(0x4D, bq.b0);
	tmc4671_writeRegister(0x4E, 29);
	tmc4671_writeRegister(0x4D, bq.b1);
	tmc4671_writeRegister(0x4E, 30);
	tmc4671_writeRegister(0x4D, bq.b2);
	tmc4671_writeRegister(0x4E, 31);
	tmc4671_writeRegister(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the pos target
 * Set nullptr to disable
 */
void TMC4671_Driver::setBiquadPos(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.pos = filter;
	tmc4671_writeRegister(0x4E, 1);
	tmc4671_writeRegister(0x4D, bq.a1);
	tmc4671_writeRegister(0x4E, 2);
	tmc4671_writeRegister(0x4D, bq.a2);
	tmc4671_writeRegister(0x4E, 4);
	tmc4671_writeRegister(0x4D, bq.b0);
	tmc4671_writeRegister(0x4E, 5);
	tmc4671_writeRegister(0x4D, bq.b1);
	tmc4671_writeRegister(0x4E, 6);
	tmc4671_writeRegister(0x4D, bq.b2);
	tmc4671_writeRegister(0x4E, 7);
	tmc4671_writeRegister(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the actual measured velocity
 * Set nullptr to disable
 */
void TMC4671_Driver::setBiquadVel(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.vel = filter;
	tmc4671_writeRegister(0x4E, 9);
	tmc4671_writeRegister(0x4D, bq.a1);
	tmc4671_writeRegister(0x4E, 10);
	tmc4671_writeRegister(0x4D, bq.a2);
	tmc4671_writeRegister(0x4E, 12);
	tmc4671_writeRegister(0x4D, bq.b0);
	tmc4671_writeRegister(0x4E, 13);
	tmc4671_writeRegister(0x4D, bq.b1);
	tmc4671_writeRegister(0x4E, 14);
	tmc4671_writeRegister(0x4D, bq.b2);
	tmc4671_writeRegister(0x4E, 15);
	tmc4671_writeRegister(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the torque target
 * Set nullptr to disable
 */
void TMC4671_Driver::setBiquadTorque(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.torque = filter;
	tmc4671_writeRegister(0x4E, 17);
	tmc4671_writeRegister(0x4D, bq.a1);
	tmc4671_writeRegister(0x4E, 18);
	tmc4671_writeRegister(0x4D, bq.a2);
	tmc4671_writeRegister(0x4E, 20);
	tmc4671_writeRegister(0x4D, bq.b0);
	tmc4671_writeRegister(0x4E, 21);
	tmc4671_writeRegister(0x4D, bq.b1);
	tmc4671_writeRegister(0x4E, 22);
	tmc4671_writeRegister(0x4D, bq.b2);
	tmc4671_writeRegister(0x4E, 23);
	tmc4671_writeRegister(0x4D, bq.enable & 0x1);
}


/**
 * Changes the torque biquad filter
 */
void TMC4671_Driver::setTorqueFilter(TMC4671Biquad_conf& conf){
//	this->torqueFilter = params;
//	setBiquadTorque(makeLpTmcFilter(params,enable));
//
	// Presets: off, Lowpass, notch, peak
	this->torqueFilterConf = conf;
	TMC4671Biquad filter;
	switch(conf.mode){
	default:
	case TMCbiquadpreset::none:
		filter = TMC4671Biquad(false);
		break;
	case TMCbiquadpreset::lowpass:
		filter = TMC4671Biquad(Biquad(BiquadType::lowpass, (float)350.0 / (float)((4.0*25e6)/(pwmCnt+1)), (float)conf.params.q/100.0,0.0), true);
		break;
	case TMCbiquadpreset::notch:
		filter = TMC4671Biquad(Biquad(BiquadType::notch, (float)350.0 / (float)((4.0*25e6)/(pwmCnt+1)), (float)conf.params.q/10.0,0.0), true);
		break;
	case TMCbiquadpreset::peak:
		filter = TMC4671Biquad(Biquad(BiquadType::peak, (float)350.0 / (float)((4.0*25e6)/(pwmCnt+1)), (float)conf.params.q/10.0,conf.gain), true);
		break;
	}
	setBiquadTorque(filter);
}

bool TMC4671_Driver::hasPower(){
//	uint16_t intV = getIntV();
//	return (intV > 10000) && (getExtV() > 10000) && (intV < 78000);
}

void TMC4671_Driver::init()
{
	setDriverState(DRIVER_DISABLE);
	// Ping driver
	tmc4671_writeRegister(TMC4671_CHIPINFO_ADDR, 0);
	if (tmc4671_readRegister(TMC4671_CHIPINFO_DATA) == 0x34363731) {
		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
	}
    else {
    	// TODO: error handler
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		setDriverState(DRIVER_DISABLE);
	  return;
	}

	// Check TMC version if it is not ES
	tmc4671_writeRegister(TMC4671_CHIPINFO_ADDR, 1);
	if(tmc4671_readRegister(TMC4671_CHIPINFO_DATA) == 0x00010000){
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
		setDriverState(DRIVER_DISABLE);
		return;
	}

	// Setup main constants
//	setDriverState(DRIVER_ENABLE);
//	setPidPrecision(&pidPrecision);
	tmc4671_writeRegister(TMC4671_PID_TORQUE_FLUX_TARGET, 0);
	setPWM(PwmMode::off ,pwmCnt, bbmL, bbmH);
	setMotorTypeAndPoles(motorType, encoder.pole_pairs);
	setPhiEType(phiEType);
	setHallConfig(&hallConfig); //enables hall filter and masking
	initAdc(&adcConfig);
//	setBiquadFlux(TMC4671Biquad(Biquad(BiquadType::lowpass, (float)350.0 / (float)((4.0*25e6)/(pwmCnt+1)), 0.7,0.0), true)); // Create flux filter

	if(!calibrateAdcOffset(300)){
		// ADC or shunt amp is broken!
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		setDriverState(DRIVER_DISABLE);
		return;
	}

	setAdcBrakeLimits(brakeLimLow, brakeLimHigh);
//	/*
//	 * Enable adc clipping and pll errors
//	 */
	statusMask.asInt = 0;
	statusMask.flags.adc_i_clipped = 1;
	statusMask.flags.not_PLL_locked = 1;
	setStatusMask(statusMask);

	setPids(&pidConfig);
	uint8_t flags = tmc4671_readRegister(TMC4671_STATUS_FLAGS);
	this->statusFlags.asInt = flags;

	// setting SD_STP (DIO6) and SD_DIR (DIO7) to High-Z
//	HAL.IOs->config->reset(&HAL.IOs->pins->DIO6);
//	HAL.IOs->config->reset(&HAL.IOs->pins->DIO7);
//
	setDriverState(DRIVER_ENABLE);
	setPWM(PwmMode::PWM_FOC);

	while(!encoder.isAligned){
		setupAbnEncoder(&encoder);
	}

//	setMotionMode(MotionMode::stopped);
	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	HAL_Delay(400);
	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
}

void TMC4671_Driver::deInit(void) {
	setDriverState(DRIVER_DISABLE);
}

uint32_t TMC4671_Driver::rotate(int32_t velocity) {
	// switch to velocity motion mode
	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_VELOCITY);

	// set target velocity for ramp generator
//	rampGenerator.targetVelocity = velocity;

	// remember switched motion mode
//	actualMotionMode = TMC4671_MOTION_MODE_VELOCITY;

	return TMC_ERROR_NONE;
}

uint32_t TMC4671_Driver::moveTo(int32_t position) {
//	// scale target position
//	position = (float) position * (float) POSITION_SCALE_MAX
//			/ (float) motorConfig.positionScaler;
//
//	// switch to position motion mode
//	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);
//
//	// set target position for ramp generator
//	rampGenerator.targetPosition = position;
//
//	// remember switched motion mode
//	actualMotionMode = TMC4671_MOTION_MODE_POSITION;
//
	return TMC_ERROR_NONE;
}

void TMC4671_Driver::moveBy(int16_t angle) {
//	// scale position deviation
//	int32_t dX = (float) *ticks * (float) POSITION_SCALE_MAX
//			/ (float) motorConfig.positionScaler;
//
//	// switch to position motion mode
//	tmc4671_switchToMotionMode(TMC4671_MOTION_MODE_POSITION);
//
//	// set target position for ramp generator
//	rampGenerator.targetPosition = tmc4671_readRegister(
//			TMC4671_PID_POSITION_ACTUAL) + dX;
//
//	// remember switched motion mode
//	actualMotionMode = TMC4671_MOTION_MODE_POSITION;
//
	// Start angle offsets all angles later so there is no jump if angle is already properly aligned
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
	const int32_t startAngle = getPhiE_Enc();
	const int32_t targetAngle = ((float)(angle*POSITION_SCALE_MAX)/360.0);

	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();
	setFluxTorque(0, 0);
	setPhiEType(PhiE::ext);
	setPhiE_ext(startAngle);

	// Ramp up flux
	for(int16_t flux = 0; flux <= 3*this->initPower/4; flux+=20){
		setFluxTorque(flux, 0);
		HAL_Delay(2);
	}

	if(angle > 0){
		for(int16_t curr_angle = 0; curr_angle <= targetAngle; curr_angle+=0x00ff){
			setPhiE_ext(curr_angle+startAngle);
			HAL_Delay(5);
		}
	}
	else {
		for(int16_t curr_angle = 0; curr_angle >= targetAngle; curr_angle-=0x00ff){
			setPhiE_ext(curr_angle+startAngle);
			HAL_Delay(5);
		}
	}
	setFluxTorque(0, 0);
	setPhiE_ext(0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);
}

void TMC4671_Driver::setMoveBy(bool state, int16_t angle){
	this->moveAngle = angle;
	this->moveFlag = state;
}
void TMC4671_Driver::periodicJob() {
	if(this->moveFlag){
		moveBy(45);
		moveFlag = false;
	}
//    tmc4671_periodicJob(motorConfig.initMode, &(motorConfig.initState), motorConfig.initWaitTime,
//                        &(motorConfig.actualInitWaitTime), motorConfig.startVoltage,
//                        &(motorConfig.hall_phi_e_old), &(motorConfig.hall_phi_e_new),
//                        &(motorConfig.hall_actual_coarse_offset),
//                        &(motorConfig.last_Phi_E_Selection), &(motorConfig.last_UQ_UD_EXT),
//                        &(motorConfig.last_PHI_E_EXT));
//
//    // 1ms velocity ramp handling
//    motorConfig.actualVelocityPT1 = tmc_filterPT1( &motorConfig.akkuActualVelocity, tmc4671_getActualVelocity(),
//												   motorConfig.actualVelocityPT1, 3, 8);
//
//	// filter actual current
//	int32_t
//	actualCurrentRaw = tmc4671_fieldRead(TMC4671_PID_TORQUE_ACTUAL_FIELD);
//	if ((actualCurrentRaw > -32000) && (actualCurrentRaw < 32000)) {
//		int32_t actualCurrent = ((int32_t) actualCurrentRaw
//				* (int32_t) motorConfig.torqueMeasurementFactor) / 256;
//		motorConfig.actualTorquePT1 = tmc_filterPT1(
//				&motorConfig.akkuActualTorque, actualCurrent,
//				motorConfig.actualTorquePT1, 4, 8);
//	}
//
//	// filter actual flux
//	int32_t
//	actualFluxRaw = tmc4671_fieldRead(TMC4671_PID_FLUX_ACTUAL_FIELD);
//	if ((actualFluxRaw > -32000) && (actualFluxRaw < 32000)) {
//		int32_t actualFlux = ((int32_t) actualFluxRaw
//				* (int32_t) motorConfig.torqueMeasurementFactor) / 256;
//		motorConfig.actualFluxPT1 = tmc_filterPT1(
//				&motorConfig.akkuActualFlux, actualFlux,
//				motorConfig.actualFluxPT1, 2, 8);
//	}
//	// do velocity / position ramping
//	if (actualMotionMode == TMC4671_MOTION_MODE_POSITION) {
//		TMC_LinearRamp_computeRampPosition(&rampGenerator);
//
//		// set new target position (only if changed)
//		if (rampGenerator.rampPosition != lastRampTargetPosition) {
//			tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
//					rampGenerator.rampPosition);
//			lastRampTargetPosition = rampGenerator.rampPosition;
//
//			// use velocity feed forward
//			tmc4671_writeRegister(TMC4671_PID_VELOCITY_OFFSET,
//					(motorConfig.enableVelocityFeedForward) ?
//							rampGenerator.rampVelocity : 0);
//		}
//
//		// sync ramp velocity by PIDIN_TARGET_VELOCITY if ramp is disabled
//		if (!rampGenerator.rampEnabled) {
//			rampGenerator.rampVelocity = tmc4671_readFieldWithDependency(TMC4671_PIDIN_TARGET_VELOCITY_FIELD, TMC4671_INTERIM_ADDR, 2);
//		}
//	}
//	else if (actualMotionMode == TMC4671_MOTION_MODE_VELOCITY) {
//		TMC_LinearRamp_computeRampVelocity(&rampGenerator);
//
//		// set new target velocity (only if changed)
//		if (rampGenerator.rampVelocity != lastRampTargetVelocity) {
//			// set new target velocity
//			tmc4671_writeRegister(TMC4671_PID_VELOCITY_TARGET,
//					rampGenerator.rampVelocity);
//			lastRampTargetVelocity = rampGenerator.rampVelocity;
//
//			// turn of velocity feed forward
//			tmc4671_writeRegister(TMC4671_PID_VELOCITY_OFFSET, 0);
//		}
//
//		// keep position ramp and target position on track
//		tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
//				tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL));
//		rampGenerator.rampPosition = tmc4671_readRegister(
//				TMC4671_PID_POSITION_ACTUAL);
//		rampGenerator.lastdXRest = 0;
//	}
//	else if (actualMotionMode == TMC4671_MOTION_MODE_TORQUE) {
//		// keep position ramp and target position on track
//		tmc4671_writeRegister(TMC4671_PID_POSITION_TARGET,
//				tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL));
//		rampGenerator.rampPosition = tmc4671_readRegister(
//				TMC4671_PID_POSITION_ACTUAL);
//		rampGenerator.rampVelocity = tmc4671_getActualVelocity();
//		rampGenerator.lastdXRest = 0;
//	}
}

void TMC4671_Driver::resetAllRegisters()
{
}
