#include "debug_led.h"

#include "hardware/sync.h"

#include "module_state.h"
#include "config.h"

namespace certainty {

void debugLedInit() {
#ifdef LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef PIN_LED_R
  pinMode(PIN_LED_R, OUTPUT);
#endif
#ifdef PIN_LED_G
  pinMode(PIN_LED_G, OUTPUT);
#endif
#ifdef PIN_LED_B
  pinMode(PIN_LED_B, OUTPUT);
#endif
}

void debugLedSet(bool on) {
#ifdef PIN_LED_R
  digitalWrite(PIN_LED_R, on ? HIGH : LOW);
#endif
#ifdef PIN_LED_G
  digitalWrite(PIN_LED_G, on ? HIGH : LOW);
#endif
#ifdef PIN_LED_B
  digitalWrite(PIN_LED_B, on ? HIGH : LOW);
#endif
#if !defined(PIN_LED_R) && !defined(PIN_LED_G) && !defined(PIN_LED_B) && defined(LED_BUILTIN)
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
#endif
}

void processI2cActivityLed(uint32_t nowMs) {
  if (!ENABLE_I2C_ACTIVITY_LED) {
    return;
  }

  static bool pulseActive = false;
  static uint32_t pulseEndMs = 0;

  if (pulseActive) {
    if ((int32_t)(nowMs - pulseEndMs) >= 0) {
      pulseActive = false;
      debugLedSet(false);
    }
    return;
  }

  if (g_module.i2cLedPulsePending == 0) {
    return;
  }

  bool startPulse = false;
  const uint32_t irqState = save_and_disable_interrupts();
  if (g_module.i2cLedPulsePending > 0) {
    g_module.i2cLedPulsePending--;
    startPulse = true;
  }
  restore_interrupts(irqState);

  if (startPulse) {
    pulseActive = true;
    pulseEndMs = nowMs + I2C_ACTIVITY_PULSE_MS;
    debugLedSet(true);
  }
}

}  // namespace certainty
