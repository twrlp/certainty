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
  volatile uint32_t gateSchedulerRuns;
  volatile uint32_t gateSchedulerMisses;

  ClockFollowerState clockFollower;
  MidiClockState     midi;
  MidiUartState      midiUart;
  repeating_timer    mainTimer;
  bool               mainTimerRunning;

  I2cRxFrame i2cRxQueue[I2C_RX_QUEUE_LEN];
  volatile uint8_t i2cRxHead;
  volatile uint8_t i2cRxTail;

  volatile uint32_t i2cRxCount;
  volatile uint32_t i2cEventQueueDropCount;
  volatile uint32_t i2cEventQueueDropTriggerCount;
  volatile uint32_t i2cEventQueueDropConfigCount;
  volatile uint32_t i2cAppliedCount;
  volatile uint32_t i2cRatioAppliedCount;
  volatile uint32_t i2cModeAppliedCount;
  volatile uint32_t i2cTriggerAppliedCount;
  volatile uint32_t i2cTriggerSaturatedCount;
  volatile uint32_t i2cErrorCount;

  // Last I2C frame diagnostics (written by processI2cEvents on Core 0)
  volatile uint8_t  dbgLastI2cLen;
  volatile uint8_t  dbgLastI2cData[I2C_RX_MAX_BYTES];
  volatile bool     dbgLastI2cDecodeOk;

  bool i2cEnabled;

  // LED feedback for I2C diagnostics
  uint32_t         ledOffMs;       // millis() when LED should turn off
  volatile bool    ledPendingOk;   // set from ISR: valid I2C decode
  volatile bool    ledPendingErr;  // set from ISR: bad I2C decode
};

extern ModuleState g_module;

// XIAO RP2040 maps GPIO6/7 to Wire (Wire0), not Wire1.
extern TwoWire &g_i2cBus;

}  // namespace certainty
