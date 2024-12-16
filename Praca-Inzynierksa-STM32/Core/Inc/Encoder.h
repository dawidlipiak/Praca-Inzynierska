/*
 * TMC4671_controller.h
 *
 *  Created on: Dec 10, 2024
 *      Author: dawid
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "inttypes.h"

enum class EncoderType : uint8_t {NONE=0,abn=1,sincos=2,uvw=3,hall=4,ext=5};

class Encoder {
public:
    Encoder();
    virtual ~Encoder();
    virtual int32_t getActualPosition();
	virtual int32_t getAbsolutePosition();
    virtual EncoderType getEncoderType();
    virtual uint32_t getCpr();
    virtual void setCpr(uint32_t cpr);
	virtual void setActualPosition(int32_t pos);

	virtual float getPos_f();
	virtual float getPosAbs_f();
    virtual bool isEncoderReady();
    virtual void setEncoderReady(bool state);
protected:
    bool encoderReady = false;
    uint32_t cpr = 40000;
};

#endif // INC_ENCODER_H_
