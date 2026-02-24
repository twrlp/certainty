#pragma once

#include <Arduino.h>

namespace certainty {

void debugLedInit();
void debugLedSet(bool on);
void processI2cActivityLed(uint32_t nowMs);

}  // namespace certainty
