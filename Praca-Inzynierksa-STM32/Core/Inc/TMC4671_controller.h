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
#include "Filters.h"

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
enum class EncoderType : uint8_t {NONE=0,abn=1,sincos=2,uvw=3,hall=4,ext=5};
enum class PosAndVelSelection: uint8_t {PhiE=0, PhiE_ext=1, PhiE_openloop=2, PhiE_abn=3, res1=4, PhiE_hal=5, PhiE_aenc=6, PhiA_aenc=7, res2=8, PhiM_abn=9, PhiM_abn2=10, PhiM_aenc=11, PhiM_hal=12};
//enum class statusFlag : uint16_t {pid_x_target_limit=0, pid_x_target_ddt_limit=1, pid_x_errsum_limit=2, pid_x_output_limit=3, pid_v_target_limit=4, pid_v_target_ddt_limit=5, pid_v_errsum_limit=6, pid_v_output_limit=7}

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

struct ABNencoder{
	uint16_t ppr 				= 10000;
	uint16_t cpr 				= 4 * ppr;
	uint8_t pole_pairs 			= 4;
	bool apol 					= false;
	bool bpol 					= false;
	bool npol					= false;
	bool rdir 					= true;
	bool ab_as_n 				= false;
	bool latch_on_N 			= false; // Restore ABN_DECODER_COUNT_N into encoder count if true on pulse. otherwise store encoder count in ABN_DECODER_COUNT_N
	int16_t phiEoffset 			= 0;	// Depends on phiM!
	int16_t phiMoffset 			= 0;
	int16_t posOffsetFromIndex 	= 0; // offset position to load after homing
	bool useIndex 				= false;
	bool isAligned 				= false;
	bool indexHitFlag			= false;
	PosAndVelSelection	posSelection = PosAndVelSelection::PhiE;
	PosAndVelSelection  velSelection = PosAndVelSelection::PhiE;
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

enum class TMCbiquadpreset : uint8_t {none=0,lowpass=1,notch=2,peak=3};
struct TMC4671Biquad_t{
	int32_t a1 = 0;
	int32_t a2 = 0;
	int32_t b0 = 0;
	int32_t b1 = 0;
	int32_t b2 = 0;
	bool enable = false;
};
struct TMC4671Biquad_conf{
	TMCbiquadpreset mode = TMCbiquadpreset::none;
	biquad_constant_t params = {1000,50}; // Q = 1/100 for lowpass and 1/10 for notch and peak mode
	float gain = 10.0; // Gain for peak mode
};

class TMC4671Biquad{
public:
	TMC4671Biquad(bool enable = false){
		params.enable = enable;
	}
	TMC4671Biquad(const TMC4671Biquad_t bq) : params(bq){}
	TMC4671Biquad(const Biquad& bq,bool enable = true){
		// Note: trinamic swapped the naming of b and a from the regular convention in the datasheet and a and b are possibly inverse to b in our filter class
		this->params.a1 = -(int32_t)(bq.b1 * (float)(1 << 29));
		this->params.a2 = -(int32_t)(bq.b2 * (float)(1 << 29));
		this->params.b0 = (int32_t)(bq.a0 * (float)(1 << 29));
		this->params.b1 = (int32_t)(bq.a1 * (float)(1 << 29));
		this->params.b2 = (int32_t)(bq.a2 * (float)(1 << 29));
		this->params.enable = bq.getFc() > 0 ? enable : false;
	}
	void enable(bool enable){
		params.enable = enable;
	}

	TMC4671Biquad_t params;
};

// Stores currently active filters
struct TMC4671BiquadFilters{
	TMC4671Biquad torque;
	TMC4671Biquad flux;
	TMC4671Biquad pos;
	TMC4671Biquad vel;
};


struct PIDConfig{
	uint16_t fluxI		= 800;
	uint16_t fluxP		= 700;
	uint16_t torqueI	= 800;
	uint16_t torqueP	= 700;
	uint16_t velocityI	= 0;
	uint16_t velocityP	= 256;
	uint16_t positionI	= 0;
	uint16_t positionP	= 128;
	bool sequentialPI	= true; // Advanced pid
};

struct PidPrecision{ // Switch between Q8.8 (false) and Q4.12 (true) precision for pid controller
	bool current_I	= false;
	bool current_P	= false;
	bool velocity_I	= false;
	bool velocity_P	= false;
	bool position_I	= false;
	bool position_P	= false;
};

struct TMC4671Limits{
	uint16_t pid_torque_flux_ddt	= 32767;
	uint16_t pid_uq_ud				= 30000;
	uint16_t pid_torque_flux		= 30000;
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

class TMC4671_Driver {
public:
	void init();
	void deInit();
	uint32_t rotate(int32_t velocity);
	uint32_t moveTo(int32_t position);
	void moveBy(int16_t angle); // +- 0-360 degrees
	void periodicJob();
	void setMoveBy(bool state);

private:
	HallConfig hallConfig;
	PIDConfig pidConfig;
	TMC4671Limits limits;
	AdcConfig adcConfig;
	ABNencoder encoder;
	PidPrecision pidPrecision;
	TMC4671Biquad_conf torqueFilterConf;
	TMC4671BiquadFilters curFilters;

	MotorType motorType			= MotorType::BLDC;
	PhiE phiEType 				= PhiE::ext;
	PwmMode pwmMode 			= PwmMode::off;
	MotionMode curr_motionMode	= MotionMode::stopped;
	EncoderType encoderType		= EncoderType::abn;
	DriverState driverState 	= DRIVER_DISABLE;
	uint16_t pwmCnt 			= 4095;
	uint8_t bbmL				= 50;
	uint8_t bbmH				= 50;
	uint16_t brakeLimLow 		= 50700;
	uint16_t brakeLimHigh 		= 50900;
	int16_t initPower 			= 9000; // Default current in setup routines ~ 7A
	StatusFlags statusFlags 	= {0};
	StatusFlags statusMask 		= {0};

	bool moveFlag				= false;
	int16_t moveAngle			= 0;

	void setDriverState(DriverState state);
	void setMotionMode(MotionMode mode);
	void setMotorTypeAndPoles(MotorType motor, uint16_t poles);
	void setPhiEType(PhiE phiEType);
	void setHallConfig(HallConfig* hallConfig);
	void setPWM(PwmMode pwmMode);
	void setPWM(PwmMode pwmMode,uint16_t maxcnt,uint8_t bbmL,uint8_t bbmH);
	void setAdcOffset(AdcConfig* adcConfig_p);
	void setAdcScale(AdcConfig* adcConfig_p);
	void initAdc(AdcConfig* adcConfig_p);
	void setPids(PIDConfig* pidConfig);
	void setPidPrecision(PidPrecision* pidPrecision);
	void setAdcBrakeLimits(uint16_t low,uint16_t high);
	void setActualPosition(int32_t pos);
	void setFluxTorque(int16_t flux, int16_t torque);
	void setStatusFlags(StatusFlags flag);
	void setStatusMask(StatusFlags mask);
	void setUdUq(int16_t ud,int16_t uq);
	void setPhiE_ext(int16_t phiE);
	void setOpenLoopSpeedAccel(int32_t speed,uint32_t accel);
	void runOpenLoop(uint16_t ud,uint16_t uq,int32_t speed,int32_t accel,bool torqueMode);
	void setBiquadFlux(const TMC4671Biquad &filter);
	void setBiquadPos(const TMC4671Biquad &filter);
	void setBiquadVel(const TMC4671Biquad &filter);
	void setBiquadTorque(const TMC4671Biquad &filter);
	void setTorqueFilter(TMC4671Biquad_conf& conf);

//	void setBiquadFlux(const TMC4671Biquad &filter);

	MotionMode getMotionMode();
	PhiE getPhiEType();
	int16_t getPhiE();
	int16_t getPhiE_Enc();
	int32_t getActualPosition();
	int32_t getAbsolutePosition();

	bool hasPower();
	bool calibrateAdcOffset(uint16_t time);
	void estimateABNparams();
	void zeroAbnUsingPhiM(bool offsetPhiE);
	void powerInitEncoder(int16_t power);
	bool checkEncoder();
	void setupAbnEncoder(ABNencoder* abnEncoder);
	void resetAllRegisters();
};


//void TMC4671_controller_enableDriver(DriverState state);
//
//void TMC4671_controller_init();
//
//void TMC4671_controller_deInit(void);
//
//uint32_t TMC4671_controller_rotate(int32_t velocity);
//
//uint32_t TMC4671_controller_moveTo(int32_t position);
//
//uint32_t TMC4671_controller_moveBy(int32_t *ticks);
//
//uint8_t TMC4671_controller_positionReached(int32_t targetPosition,
//		int32_t actualPosition, int32_t actualVelocity, int32_t maxPosDiff,
//		int32_t maxVel);
//
//void TMC4671_controller_periodicJob(void);

#endif /* INC_TMC4671_CONTROLLER_H_ */