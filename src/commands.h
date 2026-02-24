#pragma once

#include "types.h"

namespace certainty {

void applyI2cEvent(const I2cEvent &event);
void processDueConfigChanges();

}  // namespace certainty
