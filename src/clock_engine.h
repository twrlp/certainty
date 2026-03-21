#pragma once
#include <Arduino.h>
#include "pico/time.h"

namespace certainty {
bool mainCallback(repeating_timer *rt);
}  // namespace certainty
