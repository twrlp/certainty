#pragma once
#include <Arduino.h>
#include "types.h"

namespace certainty {
bool sampleLoopRetrigger(uint8_t probPercent);
float randFloat01();
float gaussianWhiteNoise();
float pinkNoiseSample(PinkNoiseState &state);
void initPinkNoise(PinkNoiseState &state);
}
