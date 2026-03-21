#pragma once
#include <Arduino.h>

namespace certainty {
void initMidiClock();
void onMidiMessage(uint8_t msg, uint64_t nowUs);
}  // namespace certainty
