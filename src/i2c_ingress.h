#pragma once

#include <Arduino.h>

namespace certainty {

void i2cReceiveHandler(int bytesCount);
void i2cRequestHandler();

void processI2cEvents();
void tryInitI2cBpmReceiver();

}  // namespace certainty
