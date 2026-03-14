#pragma once
#include <Arduino.h>
#include "pico/time.h"

namespace certainty {
void initMidiClock();
void onMidiMessage(uint8_t msg, uint64_t nowUs);
bool mainCallback(repeating_timer *rt);
}  // namespace certainty
