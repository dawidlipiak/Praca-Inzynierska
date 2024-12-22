/*
 * TMC4671_controller.h
 *
 *  Created on: Oct 27, 2024
 *      Author: dawid
 */

#ifndef INC_TMC4671_CONTROLLER_H_
#define INC_TMC4671_CONTROLLER_H_

#include <stdlib.h>
#include "stdbool.h"
#include <stdint.h>
#include <memory>

#include "stm32f4xx_hal.h"
#include "TMC4671_API.h"
#include "gpio.h"
#include "util_functions.h"
#include "Encoder.h"

#define TORQUE_FLUX_MAX 	(int32_t)10000
#define POSITION_SCALE_MAX  (int32_t)65536
#define TMC_ADCOFFSETFAIL 5000
#define MAX_REGISTER_ADDRESS 0x7F


typedef enum {
	DRIVER_DISABLE, DRIVER_ENABLE, DRIVER_USE_GLOBAL_ENABLE
} DriverState;

enum class MotorType : uint8_t {NONE = 0, DC = 1, STEPPER = 2, BLDC = 3, ERR};
enum class PhiE : uint8_t {ext = 1, openloop = 2, abn = 3, hall = 5, aenc = 6, aencE =7 , NONE, extEncoder};
enum class PwmMode : uint8_t {off = 0,HSlow_LShigh = 1, HShigh_LSlow = 2, res2 = 3, res3 = 4, PWM_LS = 5, PWM_HS = 6, PWM_FOC = 7};
enum class MotionMode : uint8_t {stopped = 0, torque = 1, velocity = 2, position = 3, prbsflux = 4, prbstorque = 5, prbsvelocity = 6, uqudext = 8, encminimove = 9, NONE};
enum class PosAndVelSelection: uint8_t {PhiE=0, PhiE_ext=1, PhiE_openloop=2, PhiE_abn=3, res1=4, PhiE_hal=5, PhiE_aenc=6, PhiA_aenc=7, res2=8, PhiM_abn=9, PhiM_abn2=10, PhiM_aenc=11, PhiM_hal=12};

// Mapping of bits in status flag register and mask
union StatusFlags {
	struct StatusFlags_s {
		uint32_t pid_x_target_limit : 1,
		pid_x_target_ddt_limit : 1,
		pid_x_errsum_limit : 1,
		pid_x_output_limit : 1,
		pid_v_target_limit : 1,
		pid_v_target_ddt_limit : 1,
		pid_v_errsum_limit : 1,
		pid_v_output_limit : 1,
		pid_id_target_limit : 1,
		pid_id_target_ddt_limit : 1,
		pid_id_errsum_limit : 1,
		pid_id_output_limit : 1,
		pid_iq_target_limit : 1,
		pid_iq_target_ddt_limit : 1,
		pid_iq_errsum_limit : 1,
		pid_iq_output_limit : 1,
		ipark_cirlim_limit_u_d : 1,
		ipark_cirlim_limit_u_q : 1,
		ipark_cirlim_limit_u_r : 1,
		not_PLL_locked : 1,
		ref_sw_r : 1,
		ref_sw_h : 1,
		ref_sw_l : 1,
		res1:1,
		pwm_min : 1,
		pwm_max : 1,
		adc_i_clipped : 1,
		adc_aenc_clipped : 1,
		ENC_N : 1,
		ENC2_N : 1,
		AENC_N : 1,
		wd_err : 1;
	};
    uint32_t asInt;
    StatusFlags_s flags;
};

struct AdcConfig{
	uint16_t mdecA 			= 660; // 334 default. 331 recommended by datasheet,662 double. 660 lowest noise
	uint16_t mdecB 			= 660; // Encoder ADC high resolution recommended
	uint32_t mclkA			= 0x20000000; //0x20000000 default
	uint32_t mclkB			= 0x20000000; // For AENC
	uint16_t adc_I0_offset 	= 33415;
	uint16_t adc_I1_offset 	= 33415;
	uint16_t adc_I0_scale	= 256;
	uint16_t adc_I1_scale	= 256;
	bool adcCalibrated		= false;
};

struct HallConfig{
	bool hallEnabled 		= false;
	bool polarity 			= true;
	bool interpolation 		= true;
	bool direction 			= false;
	bool pwmSamplingFilter 	= true;
	uint16_t blank 			= 100;
	int16_t pos0 			= 0;
	int16_t pos60 			= 10922;
	int16_t pos120 			= 21845;
	int16_t pos180 			= -32768;
	int16_t pos240			= -21846;
	int16_t pos300 			= -10923;
	int16_t phiEoffset 		= 0;
	int16_t phiMoffset 		= 0;
	uint16_t dPhiMax 		= 10922;
};

struct ABNencoderConfig{
	uint16_t ppr 				= 10000;
	uint16_t cpr 				= 40000;
	uint8_t pole_pairs 			= 4;
	bool apol 					= true;
	bool bpol 					= true;
	bool npol					= true;
	bool rdir 					= false;
	bool ab_as_n 				= false;
	bool latch_on_N 			= false; // Restore ABN_DECODER_COUNT_N into encoder count if true on pulse. otherwise store encoder count in ABN_DECODER_COUNT_N
	int16_t phiEoffset 			= 0;	// Depends on phiM!
	int16_t phiMoffset 			= 0;
	int16_t posOffsetFromIndex 	= 0; // offset position to load after homing
	bool isAligned 				= false;
	PosAndVelSelection	posSelection = PosAndVelSelection::PhiE;
	PosAndVelSelection  velSelection = PosAndVelSelection::PhiE;
};

struct PIDConfig{
	uint16_t fluxP		= 650;
	uint16_t fluxI		= 1540;
	uint16_t torqueP	= 650;
	uint16_t torqueI	= 1540;
	uint16_t velocityP	= 256;
	uint16_t velocityI	= 0;
	uint16_t positionP	= 128;
	uint16_t positionI	= 0;
	bool sequentialPI	= true; // Advanced pid
};

struct PIDLimits{
	uint16_t pid_torque_flux_ddt	= 32767;
	uint16_t pid_uq_ud				= 50000;
	uint16_t pid_torque_flux		= 50000;
	uint32_t pid_acc_lim			= 2147483647;
	uint32_t pid_vel_lim			= 2147483647;
	int32_t pid_pos_low				= -2147483647;
	int32_t pid_pos_high			= 2147483647;
};

typedef enum {
	TMC_ERROR_NONE 		= 0x00,
	TMC_ERROR_GENERIC 	= 0x01,
	TMC_ERROR_FUNCTION 	= 0x02,
	TMC_ERROR_MOTOR 	= 0x08,
	TMC_ERROR_VALUE 	= 0x10,
	TMC_ERROR_CHIP 		= 0x40
} TMCError;

class TMC4671_Driver : Encoder {
public:
	TMC4671_Driver();
	void init();
	void deInit();
	uint32_t rotate(int32_t velocity);
	uint32_t moveTo(int32_t position);
	void turn(int16_t power);
	void moveByAngle(int16_t angle); // +- 0-360 degrees
	void setMoveAngleFlag(bool state, int16_t angle);
	void periodicJob();

	// Encoder override functions
	Encoder* getEncoder();
	void setCpr(uint32_t cpr) override;
	void setActualPosition(int32_t pos) override;
	int32_t getActualPosition() override;
	int32_t getAbsolutePosition() override;
	EncoderType getEncoderType() override;

	void setTorqueLimit(uint16_t limit);
	void setPidLimits(PIDLimits limits);

	bool isInitialized();

private:
	HallConfig hallConfig;
	PIDConfig pidConfig;
	PIDLimits pidLimits;
	AdcConfig adcConfig;
	ABNencoderConfig encoderConfig;

	MotorType motorType			= MotorType::BLDC;
    EncoderType encoderType 	= EncoderType::abn;
	PhiE phiEType 				= PhiE::ext;
	PwmMode pwmMode 			= PwmMode::off;
	MotionMode curr_motionMode	= MotionMode::stopped;
	MotionMode last_motionMode	= MotionMode::stopped;
	DriverState driverState 	= DRIVER_DISABLE;
	uint16_t pwmCnt 			= 4095;
	uint8_t bbmL				= 50;
	uint8_t bbmH				= 50;
	uint16_t brakeLimLow 		= 0; //50700;
	uint16_t brakeLimHigh 		= 0; //50900;
	int16_t initPower 			= 7000; // Default current in setup routines ~ 7A
	StatusFlags statusFlags 	= {0};
	StatusFlags statusMask 		= {0};
	volatile bool moveFlag		= false;
	volatile int16_t moveAngle	= 0;
	bool isDriverInitialized	= false;

	void setupEncoder();
	void estimateABNparams();
	bool checkEncoder();
	void powerInitEncoder(int16_t power);
	void zeroAbnUsingPhiM(bool offsetPhiE = false);
	bool calibrateAdcOffset(uint16_t time);

	void setDriverState(DriverState state);
	void setMotorTypeAndPoles(MotorType motor, uint16_t poles);
	void setHallConfig(HallConfig* hallConfig);
	void setPWM(PwmMode pwmMode);
	void setPWM(PwmMode pwmMode,uint16_t maxcnt,uint8_t bbmL,uint8_t bbmH);
	void setPids(PIDConfig* pidConfig);
	void setAdcBrakeLimits(uint16_t low,uint16_t high);
	void initAdc(AdcConfig* adcConfig_p);
	void setAdcOffset(AdcConfig* adcConfig_p);
	void setAdcScale(AdcConfig* adcConfig_p);

	void setMotionMode(MotionMode mode);
	MotionMode getMotionMode();

	void setPhiEType(PhiE phiEType);
	PhiE getPhiEType();
	void setPhiE_ext(int16_t phiE);
	int16_t getPhiE_Enc();
	int16_t getPhiE();

	void setFluxTorque(int16_t flux, int16_t torque);
    void setTargetVelocity(int32_t vel);
    void setTargetPos(int32_t pos);

    void setStatusFlags(StatusFlags flag);
	void setStatusMask(StatusFlags mask);
    StatusFlags readFlags(bool maskedOnly);
};

#endif /* INC_TMC4671_CONTROLLER_H_ */
