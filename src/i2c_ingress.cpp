#include "i2c_ingress.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/regs/intctrl.h"

#include "module_state.h"
#include "commands.h"
#include "config.h"
#include "types.h"

namespace certainty {

// 1-indexed: outRaw 1..NUM_OUTPUTS maps to internal output[0..NUM_OUTPUTS-1].
static bool decodeOutIndex(uint8_t outRaw, uint8_t *outIndex) {
  if (outRaw >= 1 && outRaw <= NUM_OUTPUTS) {
    *outIndex = outRaw - 1;
    return true;
  }
  return false;
}

static bool decodeI2cEvent(const uint8_t *raw, uint8_t len, I2cEvent *event) {
  if (len == 0 || event == nullptr) {
    return false;
  }

  // cmd 0 = set BPM, with either:
  // - [0, bpm8] via IISB1
  // - [0, bpm_hi, bpm_lo] via IIS1
  if (raw[0] == 0x00) {
    uint16_t bpm = 0;
    if (len == 2) {
      bpm = raw[1];
    } else if (len >= 3) {
      bpm = (uint16_t)(((uint16_t)raw[1] << 8) | raw[2]);
    } else {
      return false;
    }

    if ((uint32_t)bpm < MIN_BPM || (uint32_t)bpm > MAX_BPM) {
      return false;
    }

    event->type = I2C_EVENT_SET_BPM;
    event->out = 0;
    event->a = 0;
    event->b = 0;
    event->mask = 0;
    event->count = bpm;
    return true;
  }

  // cmd 1 = set one output ratio: [1, out, num, den]
  if (raw[0] == 0x01 && len >= 4) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t num = raw[2];
    const uint8_t den = raw[3];
    if (num == 0 || den == 0) {
      return false;
    }

    event->type = I2C_EVENT_SET_RATIO;
    event->out = outIndex;
    event->a = num;
    event->b = den;
    event->mask = 0;
    event->count = 0;
    return true;
  }

  // cmd 2 = set run mode: [2, out, run]  (0=one-shot CT.OS, 1=loop CT.CLK)
  if (raw[0] == 0x02 && len >= 3) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t runRaw = raw[2];
    if (runRaw > (uint8_t)OUTPUT_RUN_MIDI_RESET) {
      return false;
    }
    event->type  = I2C_EVENT_SET_MODE;
    event->out   = outIndex;
    event->a     = 0;
    event->b     = runRaw;
    event->mask  = 0;
    event->count = 0;
    return true;
  }

  // cmd 4 = trigger one-shot channel(s): [4, out] or [4, 0xFF, mask]
  if (raw[0] == 0x04 && len >= 2) {
    uint8_t mask = 0;
    if (raw[1] == 0xFF) {
      if (len < 3) {
        return false;
      }
      mask = raw[2];
    } else {
      uint8_t outIndex = 0;
      if (!decodeOutIndex(raw[1], &outIndex)) {
        return false;
      }
      mask = (uint8_t)(1u << outIndex);
    }

    event->type = I2C_EVENT_TRIGGER;
    event->out = 0;
    event->a = 0;
    event->b = 0;
    event->mask = mask;
    event->count = 1;
    return true;
  }

  // cmd 5 = set per-output loop retrigger probability: [5, out, prob]
  if (raw[0] == 0x05 && len >= 3) {
    uint8_t outIndex = 0;
    if (!decodeOutIndex(raw[1], &outIndex)) {
      return false;
    }
    const uint8_t prob = raw[2];
    if (prob > 100) {
      return false;
    }

    event->type = I2C_EVENT_SET_PROB;
    event->out = outIndex;
    event->a = prob;
    event->b = 0;
    event->mask = 0;
    event->count = 0;
    return true;
  }

  return false;
}

static bool enqueueI2cRxFromIsr(const I2cRxFrame &frame) {
  const uint8_t head = g_module.i2cRxHead;
  const uint8_t tail = g_module.i2cRxTail;
  const uint8_t nextHead = (uint8_t)((head + 1u) % I2C_RX_QUEUE_LEN);
  if (nextHead == tail) {
    return false;
  }

  g_module.i2cRxQueue[head] = frame;
  g_module.i2cRxHead = nextHead;
  return true;
}

static bool dequeueI2cRx(I2cRxFrame *frame) {
  const uint32_t irqState = save_and_disable_interrupts();
  const uint8_t tail = g_module.i2cRxTail;
  if (tail == g_module.i2cRxHead) {
    restore_interrupts(irqState);
    return false;
  }

  *frame = g_module.i2cRxQueue[tail];
  g_module.i2cRxTail = (uint8_t)((tail + 1u) % I2C_RX_QUEUE_LEN);
  restore_interrupts(irqState);
  return true;
}

static void configureIrqPriorities() {
  // Lower numeric value is higher IRQ priority on RP2040.
  // Set both I2C IRQ lines high; Wire on XIAO RP2040 uses I2C0.
  irq_set_priority(I2C0_IRQ, I2C_IRQ_PRIORITY);
  irq_set_priority(I2C1_IRQ, I2C_IRQ_PRIORITY);

  // Pico-time default alarm pool uses hardware alarm 3 by default (TIMER_IRQ_3).
  // Keep timer lower than I2C to reduce truncated I2C frames under load.
  irq_set_priority(TIMER_IRQ_3, TIMER_IRQ_PRIORITY);
}

void i2cReceiveHandler(int bytesCount) {
  if (bytesCount <= 0) {
    return;
  }

  uint8_t raw[I2C_RX_MAX_BYTES] = {0};
  uint8_t len = (uint8_t)bytesCount;
  if (len > I2C_RX_MAX_BYTES) {
    len = I2C_RX_MAX_BYTES;
  }

  // Consume exactly the reported frame length so decode length is stable.
  for (uint8_t i = 0; i < len; ++i) {
    const int value = g_i2cBus.read();
    raw[i] = (uint8_t)((value < 0) ? 0 : value);
  }
  for (int i = (int)len; i < bytesCount; ++i) {
    (void)g_i2cBus.read();
  }
  while (g_i2cBus.available()) {
    (void)g_i2cBus.read();
  }

  g_module.i2cRxCount++;
  if (len == 0) {
    return;
  }

  I2cRxFrame frame = {};
  frame.len = len;
  for (uint8_t i = 0; i < len; ++i) {
    frame.data[i] = raw[i];
  }
  if (!enqueueI2cRxFromIsr(frame)) {
    g_module.i2cEventQueueDropCount++;
    if (raw[0] == 0x04) {
      g_module.i2cEventQueueDropTriggerCount++;
    } else {
      g_module.i2cEventQueueDropConfigCount++;
    }
  }
}

void i2cRequestHandler() {
  // Request/response is intentionally disabled for now.
}

void processI2cEvents(uint64_t nowUs) {
  I2cRxFrame frame = {};
  I2cEvent event = {};
  while (dequeueI2cRx(&frame)) {
    // Snapshot last frame for diagnostics.
    g_module.dbgLastI2cLen = frame.len;
    for (uint8_t i = 0; i < I2C_RX_MAX_BYTES; ++i)
      g_module.dbgLastI2cData[i] = (i < frame.len) ? frame.data[i] : 0;

    if (!decodeI2cEvent(frame.data, frame.len, &event)) {
      g_module.dbgLastI2cDecodeOk = false;
      g_module.i2cErrorCount++;
      g_module.ledPendingErr = true;
      continue;
    }
    g_module.dbgLastI2cDecodeOk = true;
    g_module.ledPendingOk = true;
    applyI2cEvent(event, nowUs);
  }
}

void tryInitI2cBpmReceiver() {
  if (g_module.i2cEnabled) return;
  // Arduino Wire.begin() does not report failure; i2cEnabled signals to the
  // retry loop that init was attempted. If the bus never comes up, i2cRxCount
  // will stay 0.
  g_i2cBus.setSDA(I2C_SDA_PIN);
  g_i2cBus.setSCL(I2C_SCL_PIN);
  g_i2cBus.begin(I2C_ADDRESS);
  g_i2cBus.onReceive(i2cReceiveHandler);
  g_i2cBus.onRequest(i2cRequestHandler);
  configureIrqPriorities();
  g_module.i2cEnabled = true;
}

}  // namespace certainty
