#pragma once

#include <Arduino.h>
#include "pico/time.h"

#include "types.h"

namespace certainty {

absolute_time_t absoluteFromUs(uint64_t usSinceBoot);

uint32_t clampBpm(uint32_t bpm);
uint64_t periodFromRatio(uint64_t beatPeriodUs, Ratio ratio);
uint64_t nextBeatBoundaryAfterLocked(uint64_t nowUs);
uint64_t alignToGlobalPhaseGridAtOrAfterLocked(uint64_t atLeastUs, uint64_t periodUs);

void resetTransportLocked(uint32_t bpm, uint64_t nowUs);
void applyBpmAndReset(uint32_t bpm);

}  // namespace certainty
