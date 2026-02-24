#pragma once

#include <Arduino.h>
#include "pico/time.h"

namespace certainty {

void configurePinAsGateOutput(uint8_t pin);

void scheduleNextAlarmLocked();
void rescheduleGateAlarmLocked();
void ensureGateSchedulerRunning();

int64_t gateAlarmCallback(alarm_id_t id, void *user_data);

}  // namespace certainty
