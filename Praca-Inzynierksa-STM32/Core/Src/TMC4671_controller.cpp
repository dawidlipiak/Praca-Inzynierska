/*
 * TMC4671_controller.c
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */


#include <TMC4671_controller.h>
#include "usbd_customhid.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

// TMC4671_Driver tmc4671;

TMC4671_Driver::TMC4671_Driver() {

}

void TMC4671_Driver::init()
{	
	// setDriverState(DRIVER_DISABLE);
	// // Ping driver
	// tmc4671_writeRegister(TMC4671_CHIPINFO_ADDR, 0);
	// uint32_t chipInfo = tmc4671_readRegister(TMC4671_CHIPINFO_DATA);
	// uint8_t buffer[64]={0};
    // buffer[0] = (uint8_t)(chipInfo & 0xFF);         // LSB
    // buffer[1] = (uint8_t)((chipInfo >> 8) & 0xFF);
    // buffer[2] = (uint8_t)((chipInfo >> 16) & 0xFF);
    // buffer[3] = (uint8_t)((chipInfo >> 24) & 0xFF);
	// USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)buffer, sizeof(buffer));
	// chipInfo = 0x12345678;
	// buffer[0] = (uint8_t)(chipInfo & 0xFF);         // LSB
    // buffer[1] = (uint8_t)((chipInfo >> 8) & 0xFF);
    // buffer[2] = (uint8_t)((chipInfo >> 16) & 0xFF);
    // buffer[3] = (uint8_t)((chipInfo >> 24) & 0xFF);
	// USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)buffer, sizeof(buffer));

	// if (tmc4671_readRegister(TMC4671_CHIPINFO_DATA) == 0x34363731) {
	// 	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
	// 	HAL_Delay(200);
	// 	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
	// }
    // else {
    // 	// TODO: error handler
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// 	setDriverState(DRIVER_DISABLE);
	//   return;
	// }

	// // Check TMC version if it is not ES
	// tmc4671_writeRegister(TMC4671_CHIPINFO_ADDR, 1);
	// if(tmc4671_readRegister(TMC4671_CHIPINFO_DATA) == 0x00010000){
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// 	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
	// 	setDriverState(DRIVER_DISABLE);
	// 	return;
	// }

	// Setup main constants
	// tmc4671_writeRegister(TMC4671_PID_TORQUE_FLUX_TARGET, 0);
	setPWM(PwmMode::PWM_FOC ,3999, 25, 25);
	setMotorTypeAndPoles(motorType, 4);
	tmc4671_fieldWrite(TMC4671_POSITION_SELECTION_FIELD, (uint8_t)PosAndVelSelection::PhiE_openloop);
	tmc4671_fieldWrite(TMC4671_VELOCITY_SELECTION_FIELD, (uint8_t)PosAndVelSelection::PhiE_openloop);
	setDriverState(DRIVER_ENABLE);
	setFluxTorque(3000,0);
	int16_t oldPhiE = getPhiE();
	setPhiEType(PhiE::openloop);
	tmc4671_fieldWrite(TMC4671_OPENLOOP_PHI_DIRECTION_FIELD, 0);
	tmc4671_writeRegister(TMC4671_OPENLOOP_PHI, oldPhiE);

	// tmc4671_writeRegister(TMC4671_PWM_BBM_H_BBM_L, 0);
	tmc4671_fieldWrite(TMC4671_MODE_MOTION_FIELD, 8);
	tmc4671_fieldWrite(TMC4671_MODE_RAMP_FIELD, 0);
	tmc4671_fieldWrite(TMC4671_MODE_FF_FIELD, 0);
	tmc4671_fieldWrite(TMC4671_MODE_PID_SMPL_FIELD, 0);
	tmc4671_fieldWrite(TMC4671_MODE_PID_TYPE_FIELD, 0);
	tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, 0);


	setMotionMode(MotionMode::uqudext);
	tmc4671_fieldWrite(TMC4671_UD_EXT_FIELD, 5000);
	tmc4671_fieldWrite(TMC4671_UQ_EXT_FIELD, 0);


	// tmc4671_fieldWrite(TMC4671_OPENLOOP_VELOCITY_TARGET_FIELD, 5);
	// tmc4671_fieldWrite(TMC4671_OPENLOOP_ACCELERATION_FIELD, 10);
	tmc4671_writeRegister(TMC4671_OPENLOOP_VELOCITY_TARGET, 20);
	tmc4671_writeRegister(TMC4671_OPENLOOP_ACCELERATION, 10);
	

	// setHallConfig(&hallConfig); //enables hall filter and masking
	// initAdc(&adcConfig);

	// if(!calibrateAdcOffset(300)){
	// 	// ADC or shunt amp is broken!
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// 	setDriverState(DRIVER_DISABLE);
	// 	return;
	// }

	// setAdcBrakeLimits(brakeLimLow, brakeLimHigh);

	// // Enable adc clipping and pll errors
	// statusMask.asInt = 0;
	// statusMask.flags.adc_i_clipped = 1;
	// statusMask.flags.not_PLL_locked = 1;
	// setStatusMask(statusMask);

	// setPids(&pidConfig);
	// setPidLimits(pidLimits);
	// uint8_t flags = tmc4671_readRegister(TMC4671_STATUS_FLAGS);
	// this->statusFlags.asInt = flags;

	// setDriverState(DRIVER_ENABLE);
	// setPWM(PwmMode::PWM_FOC);

	// while(!encoderConfig.isAligned){
	// 	setupEncoder();
	// }
	// setPWM(PwmMode::PWM_FOC);
	// setDriverState(DRIVER_ENABLE);
	// setMotionMode(MotionMode::stopped);
	// setPhiEType(PhiE::ext);
	// tmc4671_fieldWrite(TMC4671_POSITION_SELECTION_FIELD, (uint8_t)PosAndVelSelection::PhiE_ext);
	// tmc4671_fieldWrite(TMC4671_VELOCITY_SELECTION_FIELD, (uint8_t)PosAndVelSelection::PhiE_ext);

	// for(uint16_t vel = 50; vel <= 3000; vel+= 20){
	// 	setTargetVelocity(vel);
	// 	HAL_Delay(2);
	// }
	// setTargetVelocity(0);
	// int32_t targetVel = tmc4671_fieldRead(TMC4671_PID_VELOCITY_TARGET_FIELD);
	// if(targetVel != 1000){
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// }


	// HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// HAL_Delay(400);
	// HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
	// HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);

	// readFlags(true);
	// if (statusFlags.flags.pid_x_target_limit ||
    //     statusFlags.flags.pid_x_target_ddt_limit ||
    //     statusFlags.flags.pid_x_errsum_limit ||
    //     statusFlags.flags.pid_x_output_limit ||
    //     statusFlags.flags.pid_v_target_limit ||
    //     statusFlags.flags.pid_v_target_ddt_limit ||
    //     statusFlags.flags.pid_v_errsum_limit ||
    //     statusFlags.flags.pid_v_output_limit ||
    //     statusFlags.flags.pid_id_target_limit ||
    //     statusFlags.flags.pid_id_target_ddt_limit ||
    //     statusFlags.flags.pid_id_errsum_limit ||
    //     statusFlags.flags.pid_id_output_limit ||
    //     statusFlags.flags.pid_iq_target_limit ||
    //     statusFlags.flags.pid_iq_target_ddt_limit ||
    //     statusFlags.flags.pid_iq_errsum_limit ||
    //     statusFlags.flags.pid_iq_output_limit ||
    //     statusFlags.flags.ipark_cirlim_limit_u_d ||
    //     statusFlags.flags.ipark_cirlim_limit_u_q ||
    //     statusFlags.flags.ipark_cirlim_limit_u_r ||
    //     statusFlags.flags.not_PLL_locked ||
    //     statusFlags.flags.ref_sw_r ||
    //     statusFlags.flags.ref_sw_h ||
    //     statusFlags.flags.ref_sw_l ||
    //     statusFlags.flags.pwm_min ||
    //     statusFlags.flags.pwm_max ||
    //     statusFlags.flags.adc_i_clipped ||
    //     statusFlags.flags.adc_aenc_clipped ||
    //     statusFlags.flags.ENC_N ||
    //     statusFlags.flags.ENC2_N ||
    //     statusFlags.flags.AENC_N ||
    //     statusFlags.flags.wd_err
	// ) {
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// }
	// else {
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	// }

	// int32_t velError = tmc4671_fieldRead(TMC4671_PID_VELOCITY_ERROR_FIELD);
	
	// if(velError < 0xFFF && velError > 0xFF ){
	// 	HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
	// }
	// TMC4671_PWM_IN_RAW_FIELD
}

void TMC4671_Driver::deInit(void) {
	setDriverState(DRIVER_DISABLE);
	setMotionMode(MotionMode::stopped);
	setPWM(PwmMode::off);
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

void TMC4671_Driver::turn(int16_t power) {
	if(!this->isInitialized())
		return;
	// int32_t flux = 0;
	

	// setFluxTorque(0, power);
	// setTargetVelocity(power/2);
}

void TMC4671_Driver::moveByAngle(int16_t angle) {

	// Start angle offsets all angles later so there is no jump if angle is already properly aligned
	const int32_t startAngle = getActualPosition();
	const int32_t targetAngle = (((float)(startAngle + angle)*POSITION_SCALE_MAX)/360.0f);

	PhiE lastphie = getPhiEType();
	MotionMode lastmode = getMotionMode();
	setFluxTorque(0, 0);
	setPhiEType(PhiE::abn);

	// Ramp up flux
	for(int16_t flux = 0; flux <= this->initPower && getActualPosition() ; flux+=20){
		setFluxTorque(flux, flux);
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

void TMC4671_Driver::setMoveAngleFlag(bool state, int16_t angle){
	this->moveAngle = angle;
	this->moveFlag = state;
}

void TMC4671_Driver::periodicJob() {
	if(this->moveFlag){
		moveFlag = false;
		moveByAngle(this->moveAngle);
	}
}

Encoder* TMC4671_Driver::getEncoder(){
	return static_cast<Encoder*>(this);
}

void TMC4671_Driver::setCpr(uint32_t cpr){
	if(cpr == 0)
		cpr = 1;
	this->encoderConfig.cpr = cpr;
	this->cpr = cpr;
}

// Changes actual multi turn position for positioning
void TMC4671_Driver::setActualPosition(int32_t pos){
	tmc4671_writeRegister(TMC4671_PID_POSITION_ACTUAL, (uint32_t)pos);
}

// Returns actual multi turn position from tmc
int32_t TMC4671_Driver::getActualPosition(){
	// Upper 16 bits are counted revolutions of the encoder
	// Lower 16 bits are position within one rotation (0 - CPR)
	// Any position comfing from a encoder is mapped to the 2^16 (65536) steps range
	int32_t pid_position_actual = (int32_t)tmc4671_readRegister(TMC4671_PID_POSITION_ACTUAL);

	return pid_position_actual;
}

int32_t TMC4671_Driver::getAbsolutePosition(){
	int16_t pos;

	if(getEncoderType() == EncoderType::abn){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_M_FIELD);
	}
	else if(getEncoderType() == EncoderType::hall){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_HALL_PHI_M_FIELD);
	}
	else if(getEncoderType() == EncoderType::sincos || getEncoderType() == EncoderType::uvw){
		pos = (int16_t)tmc4671_fieldRead(TMC4671_AENC_DECODER_PHI_M_FIELD);
	}
	else{
		pos = getActualPosition(); // read phiM
	}

	return pos;
}

EncoderType TMC4671_Driver::getEncoderType(){
	return this->encoderType;
}

void TMC4671_Driver::setTorqueLimit(uint16_t limit){
	this->pidLimits.pid_torque_flux = limit;
	initPower = (float)limit*0.75;
	tmc4671_writeRegister(TMC4671_PID_TORQUE_FLUX_LIMITS, limit);
}

void TMC4671_Driver::setPidLimits(PIDLimits limits){
	this->pidLimits = limits;
	tmc4671_writeRegister(TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS, pidLimits.pid_torque_flux_ddt);
	tmc4671_writeRegister(TMC4671_PIDOUT_UQ_UD_LIMITS, pidLimits.pid_uq_ud);
	tmc4671_writeRegister(TMC4671_PID_TORQUE_FLUX_LIMITS, pidLimits.pid_torque_flux);
	tmc4671_writeRegister(TMC4671_PID_ACCELERATION_LIMIT,pidLimits.pid_acc_lim);
	tmc4671_writeRegister(TMC4671_PID_VELOCITY_LIMIT, pidLimits.pid_vel_lim);
	tmc4671_writeRegister(TMC4671_PID_POSITION_LIMIT_LOW, pidLimits.pid_pos_low);
	tmc4671_writeRegister(TMC4671_PID_POSITION_LIMIT_HIGH, pidLimits.pid_pos_high);
}

bool TMC4671_Driver::isInitialized()
{
	return this->isDriverInitialized;
}

void TMC4671_Driver::setupEncoder(){
	this->statusMask.flags.AENC_N = 0;
	this->statusMask.flags.ENC_N = 0;
	setStatusMask(statusMask);

	tmc4671_fieldWrite(TMC4671_ABN_APOL_FIELD, encoderConfig.apol);
	tmc4671_fieldWrite(TMC4671_ABN_BPOL_FIELD, encoderConfig.bpol);
	tmc4671_fieldWrite(TMC4671_ABN_NPOL_FIELD, encoderConfig.npol);
	tmc4671_fieldWrite(TMC4671_USE_ABN_AS_N_FIELD, encoderConfig.ab_as_n);
	tmc4671_fieldWrite(TMC4671_ABN_CLN_FIELD, encoderConfig.latch_on_N);
	tmc4671_fieldWrite(TMC4671_ABN_DIRECTION_FIELD, encoderConfig.rdir);

	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PPR_FIELD, encoderConfig.cpr);

	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD, encoderConfig.phiEoffset);
	tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_M_OFFSET_FIELD, encoderConfig.phiMoffset);

	// Set mechanical angle
	encoderConfig.posSelection = PosAndVelSelection::PhiM_abn;
	encoderConfig.velSelection = PosAndVelSelection::PhiM_abn;
	tmc4671_fieldWrite(TMC4671_POSITION_SELECTION_FIELD, (uint8_t)encoderConfig.posSelection);
	tmc4671_fieldWrite(TMC4671_VELOCITY_SELECTION_FIELD, (uint8_t)encoderConfig.velSelection);
	tmc4671_fieldWrite(TMC4671_VELOCITY_METER_SELECTION_FIELD, 0); // 0: default velocity meter (fixed frequency sampling)


	estimateABNparams();

	setPhiE_ext(tmc4671_readRegister(TMC4671_PHI_E));
	setPhiEType(PhiE::ext);

	// Align encoder
	powerInitEncoder(this->initPower);

	uint8_t enc_retry = 0;
	while(!encoderConfig.isAligned && enc_retry < 3){
		checkEncoder();
		enc_retry++;
	}
	if(!encoderConfig.isAligned) {
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		setDriverState(DRIVER_DISABLE);
		return;
	}

	if(getEncoderType() == EncoderType::abn){
		setPhiEType(PhiE::abn);
	}

	this->encoderReady = encoderConfig.isAligned;
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_SET);
//	HAL_Delay(1000);
//	HAL_GPIO_WritePin(LED_SYS_GPIO_Port, LED_SYS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1000);
//	else if(usingExternalEncoder()){
//		setPhiEType(PhiE::extEncoder);
//	}
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
	encoderConfig.rdir = rcount > c/2;

	if(npol != encoderConfig.npol){ // Invert dir if polarity was reversed TODO correct? likely wrong at the moment
		// encoderConfig.rdir = !encoderConfig.rdir;
		// HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		// HAL_Delay(300);
		// HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
	}


	encoderConfig.apol = npol;
	encoderConfig.bpol = npol;
	encoderConfig.npol = npol;

//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_SET);
//	HAL_Delay(200);
//	HAL_GPIO_WritePin(LED_CLIP_GPIO_Port, LED_CLIP_Pin, GPIO_PIN_RESET);
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
	for(int16_t flux = 0; flux <= this->initPower; flux+=20){
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

	if(revCount > maxcount){ // Encoder seems reversed
		// reverse encoder
		if(getEncoderType() == EncoderType::abn){
			// this->encoderConfig.rdir = !this->encoderConfig.rdir;
//			this->encoderConfig.apol = !this->encoderConfig.apol;
//			this->encoderConfig.bpol = !this->encoderConfig.bpol;
//			this->encoderConfig.npol = !this->encoderConfig.npol;
			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
			HAL_Delay(300);
			HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
			// tmc4671_fieldWrite(TMC4671_ABN_DIRECTION_FIELD, this->encoderConfig.rdir);
//			tmc4671_fieldWrite(TMC4671_ABN_APOL_FIELD, this->encoderConfig.apol);
//			tmc4671_fieldWrite(TMC4671_ABN_BPOL_FIELD, this->encoderConfig.bpol);
//			tmc4671_fieldWrite(TMC4671_ABN_NPOL_FIELD, this->encoderConfig.npol);
//			result = false;
//			setupEncoder(&encoder);
		}
	}

	setFluxTorque(0, 0);
	setPhiE_ext(0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);

	if(result){
		encoderConfig.isAligned = true;
	}

	return result;
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

	if(getEncoderType() == EncoderType::abn){
		phiEoffsetReg = TMC4671_ABN_DECODER_PHI_E_OFFSET_FIELD;
		zeroAbnUsingPhiM();
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

	//Write offset
	int16_t phiEoffset =  phiEpos-phiE_enc;

	if(phiEoffset == 0){ // 0 invalid
		HAL_Delay(200);
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
		HAL_Delay(5000);
		phiEoffset = 1;
	}

	tmc4671_fieldWrite(phiEoffsetReg, phiEoffset);

	if(getEncoderType() == EncoderType::abn){
		encoderConfig.phiEoffset = phiEoffset;
	}

	setPhiE_ext(0);
	setPhiEType(lastphie);
	setMotionMode(lastmode);
}

void TMC4671_Driver::zeroAbnUsingPhiM(bool offsetPhiE){
	int32_t npos = (int32_t)(TMC4671_ABN_DECODER_COUNT_N); // raw encoder counts at index hit
	int32_t npos_M = (npos * 0xffff) / encoderConfig.cpr; // Scaled encoder angle at index
	encoderConfig.phiMoffset = -npos_M;

	if(offsetPhiE){
		encoderConfig.phiEoffset += npos_M * encoderConfig.pole_pairs;
	}else{
		tmc4671_fieldWrite(TMC4671_ABN_DECODER_PHI_M_OFFSET_FIELD, encoderConfig.phiMoffset);
	}

	setActualPosition(getAbsolutePosition()); // Set position to absolute position = ~zero
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

	// Disable drivers and measure many samples of zero current
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
	}
	int32_t offsetAidle = totalA / (measurements_idle);
	int32_t offsetBidle = totalB / (measurements_idle);

	// Check if offsets are in a valid range
	if(totalA < 100 || totalB < 100 || ((abs(offsetAidle - 0x7fff) > TMC_ADCOFFSETFAIL) || (abs(offsetBidle - 0x7fff) > TMC_ADCOFFSETFAIL)) ){
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);;
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

void TMC4671_Driver::setDriverState(DriverState state) {
	if (state == DRIVER_DISABLE) {
		this->driverState = DRIVER_DISABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
	} else {
		this->driverState = DRIVER_ENABLE;
		HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
	}
}

void TMC4671_Driver::setMotorTypeAndPoles(MotorType motor, uint16_t poles){
	if(motor == MotorType::DC){
		poles = 1;
	}

	bool enableSvPwm = false; // Space vector pwm is only for 3 phase motors
	if(motor == MotorType::BLDC){
		enableSvPwm = true;
	}
	this->motorType = motor;
	this->encoderConfig.pole_pairs = poles;

	tmc4671_fieldWrite(TMC4671_N_POLE_PAIRS_FIELD, poles);
	tmc4671_fieldWrite(TMC4671_MOTOR_TYPE_FIELD, (uint8_t)motor);
	// tmc4671_fieldWrite(TMC4671_PWM_SV_FIELD, enableSvPwm);
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
	// maxcnt = clip(maxcnt, 255, 4095);
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

void TMC4671_Driver::setMotionMode(MotionMode mode){
	if(mode != curr_motionMode){
		last_motionMode = curr_motionMode;
	}
	this->curr_motionMode = mode;
	tmc4671_fieldWrite(TMC4671_MODE_MOTION_FIELD, (uint8_t) mode);
}

MotionMode TMC4671_Driver::getMotionMode(){
	this->curr_motionMode = (MotionMode) tmc4671_fieldRead(TMC4671_MODE_MOTION_FIELD);
	return this->curr_motionMode;
}

void TMC4671_Driver::setPhiEType(PhiE phiEType){
	if(phiEType == PhiE::extEncoder){
		phiEType = PhiE::ext;
	}
	this->phiEType = phiEType;

	tmc4671_fieldWrite(TMC4671_PHI_E_SELECTION_FIELD, (uint8_t)phiEType && 0xFF);
}

PhiE TMC4671_Driver::getPhiEType(){
	this->phiEType = (PhiE) (tmc4671_fieldRead(TMC4671_PHI_E_SELECTION_FIELD) & 0x07);
	return this->phiEType;
}

void TMC4671_Driver::setPhiE_ext(int16_t phiE){
	tmc4671_fieldWrite(TMC4671_PHI_E_EXT_FIELD, (uint32_t)phiE);
}

/**
 * Reads phiE directly from the encoder selection instead of the current phiE selection
 */
int16_t TMC4671_Driver::getPhiE_Enc(){
	if(getEncoderType() == EncoderType::abn){
		return (int16_t)tmc4671_fieldRead(TMC4671_ABN_DECODER_PHI_E_FIELD);
	}
	else if(getEncoderType() == EncoderType::sincos || getEncoderType() == EncoderType::uvw){
		return (int16_t)tmc4671_fieldRead(TMC4671_AENC_DECODER_PHI_E_FIELD);
	}
	else if(getEncoderType() == EncoderType::hall){
		return (int16_t)tmc4671_fieldRead(TMC4671_HALL_PHI_E_FIELD);
	}
	else{
		return getPhiE();
	}
}

int16_t TMC4671_Driver::getPhiE(){
	return (int16_t)tmc4671_readRegister(TMC4671_PHI_E);
}

void TMC4671_Driver::setFluxTorque(int16_t flux, int16_t torque){
	if(curr_motionMode != MotionMode::torque){
		setMotionMode(MotionMode::torque);
	}

	tmc4671_fieldWrite(TMC4671_PID_FLUX_TARGET_FIELD, flux);
	tmc4671_fieldWrite(TMC4671_PID_TORQUE_TARGET_FIELD, torque);
}

void TMC4671_Driver::setTargetVelocity(int32_t vel){
	if(curr_motionMode != MotionMode::velocity){
		setMotionMode(MotionMode::velocity);
	}
	tmc4671_writeRegister(TMC4671_PID_VELOCITY_TARGET,vel);
}

void TMC4671_Driver::setTargetPos(int32_t pos){
	if(curr_motionMode != MotionMode::position){
		setMotionMode(MotionMode::position);
	}
	tmc4671_writeRegister(0x68,pos);
}

void TMC4671_Driver::setStatusFlags(StatusFlags flag){
	tmc4671_writeRegister(TMC4671_STATUS_MASK, flag.asInt);
}

void TMC4671_Driver::setStatusMask(StatusFlags mask){
	tmc4671_writeRegister(TMC4671_STATUS_MASK, mask.asInt);
}

StatusFlags TMC4671_Driver::readFlags(bool maskedOnly){
	uint32_t flags = tmc4671_readRegister(0x7C);
	if(maskedOnly){
		flags = flags & this->statusMask.asInt;
	}
	this->statusFlags.asInt = flags; // Only set flags that are marked to trigger a notification
	return statusFlags;
}
