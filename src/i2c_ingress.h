#pragma once

#include <Arduino.h>

namespace certainty {

void i2cReceiveHandler(int bytesCount);
void i2cRequestHandler();

void processI2cEvents(uint64_t nowUs);
void tryInitI2cBpmReceiver();

}  // namespace certainty
