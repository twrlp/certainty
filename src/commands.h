#pragma once

#include "types.h"

namespace certainty {

void applyI2cEvent(const I2cEvent &event, uint64_t nowUs);
void processI2cEvents(uint64_t nowUs);
void processDueConfigChanges(uint64_t nowUs);

}  // namespace certainty
