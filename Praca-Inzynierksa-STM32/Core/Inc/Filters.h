/*
 * Filters.h
 *
 *  Created on: Nov 11, 2024
 *      Author: dawid
 */

#ifndef INC_FILTERS_H_
#define INC_FILTERS_H_

#include <cstdint>
#include "util_functions.h"

// Frequency in hz, q in float q*100. Example: Q 0.5 -> 50
struct biquad_constant_t {
	uint16_t freq;
	uint8_t q;
};

enum class BiquadType : uint8_t {
    lowpass = 0,
    highpass,
    bandpass,
    notch,
    peak,
    lowshelf,
    highshelf
};

class TMC4671Biquad;
class Biquad{
	friend TMC4671Biquad;
public:
	Biquad();
    Biquad(BiquadType type, float Fc, float Q, float peakGainDB);
    ~Biquad();
    float process(float in);
    void setBiquad(BiquadType type, float Fc, float Q, float peakGain);
    void setFc(float Fc); //frequency
    float getFc() const;
    void setQ(float Q);
    float getQ() const;
    void calcBiquad(void);

protected:

    BiquadType type;
    float a0, a1, a2, b1, b2;
    float Fc, Q, peakGain;
    float z1, z2;
};

#endif /* INC_FILTERS_H_ */
