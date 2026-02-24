#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "pico/time.h"

#include "config.h"
#include "types.h"

namespace certainty {

struct ModuleState {
  OutputState outputs[NUM_OUTPUTS];
  TransportState transport;
  alarm_id_t gateAlarmId;
  repeating_timer pwmTimer;
  volatile uint32_t gateSchedulerRuns;
  volatile uint32_t gateSchedulerMisses;
  volatile uint32_t pwmRuns;

  uint32_t pwmSliceMask;
  bool pwmTimerRunning;

  I2cRxFrame i2cRxQueue[I2C_RX_QUEUE_LEN];
  volatile uint8_t i2cRxHead;
  volatile uint8_t i2cRxTail;

  volatile uint32_t i2cRxCount;
  volatile uint32_t i2cEventQueueDropCount;
  volatile uint32_t i2cEventQueueDropTriggerCount;
  volatile uint32_t i2cEventQueueDropConfigCount;
  volatile uint16_t i2cLedPulsePending;

  volatile uint32_t i2cAppliedCount;
  volatile uint32_t i2cRatioAppliedCount;
  volatile uint32_t i2cModeAppliedCount;
  volatile uint32_t i2cAsrAppliedCount;
  volatile uint32_t i2cTriggerAppliedCount;
  volatile uint32_t i2cTriggerSaturatedCount;
  volatile uint32_t i2cErrorCount;

  bool i2cEnabled;
};

extern ModuleState g_module;

// XIAO RP2040 maps GPIO6/7 to Wire (Wire0), not Wire1.
extern TwoWire &g_i2cBus;

}  // namespace certainty
