#pragma once

#include <Arduino.h>

#include "config.h"

namespace certainty {

enum OutputRun : uint8_t {
  OUTPUT_RUN_ONE_SHOT = 0,
  OUTPUT_RUN_LOOP = 1,
};

struct Ratio {
  uint16_t num;
  uint16_t den;
};

struct OutputState {
  uint8_t   pin;
  Ratio     ratio;
  OutputRun run;

  uint8_t   loopProbPercent;

  bool      ratioPending;
  uint64_t  pendingApplyUs;
  Ratio     pendingRatio;
  bool      pendingRunPending;
  uint64_t  pendingRunApplyUs;
  OutputRun pendingRun;
  uint16_t  pendingTriggerCount;

  uint64_t  nextFallUs;
  bool      gateHigh;
  uint32_t  gateRises;
  uint32_t  gateFalls;

  float     phase;
  float     freq;
};

struct TransportState {
  uint32_t bpm;
  uint64_t beatPeriodUs;
  uint64_t anchorUs;      // last PPQN edge (freq computation)
  uint64_t beatAnchorUs;  // last true beat boundary
  uint32_t resetCount;
};

enum ClockFollowMode : uint8_t {
  CLK_FOLLOW_INACTIVE  = 0,
  CLK_FOLLOW_LOCKED    = 2,
};

struct ClockFollowerState {
  ClockFollowMode mode;
  uint32_t        fallbackBpm;
  uint64_t        inputEdgeCount;
};

enum MidiPlayState : uint8_t {
  MIDI_STOPPED = 0,
  MIDI_PLAYING = 1,
};

enum MidiUartPhase : uint8_t {
  MIDI_UART_IDLE         = 0,
  MIDI_UART_START_VERIFY = 1,
  MIDI_UART_DATA         = 2,
  MIDI_UART_STOP         = 3,
};

struct MidiClockState {
  MidiPlayState     playState;
  bool              startPending;        // true after 0xFA, cleared on first 0xF8
  uint64_t          lastClockUs;         // timestamp of most recent 0xF8
  uint64_t          smoothedPeriodUs;    // IIR-filtered clock interval
  uint32_t          clockCount;          // 0xF8s since last Start (resets on Start)
  uint32_t          totalClocks;         // monotonic (diagnostics)
};

struct MidiUartState {
  MidiUartPhase phase;
  uint8_t       sampleCount;
  uint8_t       bitIndex;
  uint8_t       byte;
  uint32_t      readIdx;      // position in DMA ring buffer
  int           dataChannel;  // DMA data channel (ADC → ring buffer)
  int           ctrlChannel;  // DMA control channel (reloads data channel)

  // Debug counters — written by Core 1, read/reset by Core 0 appLoop.
  // volatile makes compiler hoisting impossible; word-sized accesses are atomic
  // on RP2040, so at most one count may be lost per 500ms reporting interval —
  // acceptable for diagnostics.
  volatile uint8_t  dbgAdcMin;
  volatile uint8_t  dbgAdcMax;
  volatile uint8_t  dbgAdcAtLow;      // min ADC seen while decoding (START_VERIFY or DATA phase)
  volatile uint32_t dbgStartBits;     // IDLE→START_VERIFY transitions
  volatile uint32_t dbgFalseStarts;   // START_VERIFY→IDLE rejections (start bit went HIGH)
  volatile uint32_t dbgStopFails;     // stop bit was !mark (signal didn't recover)
  volatile uint32_t dbgByteFails;     // stop bit ok but byte < 0xF8 (bit error / wrong message)
  volatile uint32_t dbgBytesDecoded;  // complete RT bytes passed to onMidiMessage
  volatile uint8_t  dbgLastByte;      // most recent decoded byte value
  volatile uint32_t dbgRtClock;       // 0xF8
  volatile uint32_t dbgRtStart;       // 0xFA
  volatile uint32_t dbgRtContinue;    // 0xFB
  volatile uint32_t dbgRtStop;        // 0xFC
  volatile uint32_t dbgRtActiveSense; // 0xFE
  volatile uint32_t dbgMsgDrops;      // bytes dropped due to full SPSC ring buffer

  volatile uint8_t adcThreshold;      // dynamic threshold used for mark/space
  volatile uint8_t adcStartThreshold; // higher threshold for start-bit edge detect

  // Inter-core SPSC ring buffer — Core 1 writes (msgHead), Core 0 reads (msgTail).
  volatile uint8_t msgBuf[16];
  volatile uint8_t msgHead;
  volatile uint8_t msgTail;

  volatile uint8_t  dbgStartSample;    // ADC at mid start bit
  volatile uint8_t  dbgStopSample;     // ADC at mid stop bit
  volatile uint8_t  dbgBitSamples[8];  // ADC at mid of each data bit (last decoded byte)
};

enum I2cEventType : uint8_t {
  I2C_EVENT_SET_BPM      = 0,
  I2C_EVENT_SET_RATIO    = 1,
  I2C_EVENT_SET_MODE     = 2,
  I2C_EVENT_TRIGGER      = 4,
  I2C_EVENT_SET_PROB     = 5,
};

struct I2cEvent {
  I2cEventType type;
  uint8_t out;
  uint8_t a;
  uint8_t b;
  uint8_t mask;
  uint16_t count;
};

struct I2cRxFrame {
  uint8_t len;
  uint8_t data[I2C_RX_MAX_BYTES];
};

}  // namespace certainty
