#include "Encoder.h"

// Constructor
Encoder::Encoder() {}

// Destructor
Encoder::~Encoder() {}

// Get the actual position of the encoder
// Function must be override and return appropriate value
int32_t Encoder::getActualPosition() {
    return 0;
}

// Get the absolute position of the encoder
// Function must be override and return appropriate value
int32_t Encoder::getAbsolutePosition() {
    // Function implementation (empty)
    return 0;
}

// Get the encoder type
// Function must be override and return appropriate value
EncoderType Encoder::getEncoderType() {
    return EncoderType::NONE;
}

// Get the counts per revolution (CPR)
uint32_t Encoder::getCpr() {
    return this->cpr;
}

// Set the counts per revolution (CPR)
void Encoder::setCpr(uint32_t cpr) {
    this->cpr = cpr;
}

// Set the actual position of the encoder. Can be used to reset center
// Function must be override with appropriate definition
void Encoder::setActualPosition(int32_t pos) {
    // dummy
}

// Get the floating-point representation of the actual position 
float Encoder::getPos_f() {
        if(this->getCpr() == 0) {
        return 0.0;
    }

    return (float)this->getActualPosition() / (float)this->getCpr();
}

// Get the floating-point representation of the absolute position
float Encoder::getPosAbs_f() {
    if(this->getCpr() == 0) {
        return 0.0;
    }

    return (float)this->getAbsolutePosition() / (float)this->getCpr();
}

// Check if the encoder is ready
bool Encoder::isEncoderReady() {
    return this->encoderReady;
}

// Set the encoder ready state
void Encoder::setEncoderReady(bool state) {
    encoderReady = state;
}
