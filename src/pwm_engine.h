#pragma once

#include "pico/time.h"

#include "types.h"

namespace certainty {

bool usesPwm(OutputShape shape);

void configurePinAsPwmOutput(OutputState &out);
void configureOutputPinsForModes();

void deriveAsrWeights(uint8_t sus, uint8_t skew, uint8_t *a, uint8_t *s, uint8_t *r);
void refreshAsrTimingLocked(OutputState &out);

bool pwmSampleCallback(repeating_timer *rt);

}  // namespace certainty
