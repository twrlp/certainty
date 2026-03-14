#pragma once
#include <Arduino.h>

namespace certainty {
void initMidiDma();
void processMidiAdcSamples();           // called from Core 1 tight loop
void runMidiUartCore1();                // infinite loop, called from loop1()
void drainMidiMessages(uint64_t nowUs); // called from mainCallback on Core 0
}  // namespace certainty
