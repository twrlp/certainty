#include "rng.h"
#include "pico/time.h"

namespace certainty {

static uint32_t rngState = 0xA341316Cu;

static inline uint32_t xorshift32(uint32_t x) {
  x ^= x << 13u;
  x ^= x >> 17u;
  x ^= x << 5u;
  return x;
}

bool sampleLoopRetrigger(uint8_t probPercent) {
  if (probPercent >= 100) return true;
  if (probPercent == 0) return false;
  rngState = xorshift32(rngState ^ (uint32_t)time_us_64());
  return (rngState % 100u) < probPercent;
}

}  // namespace certainty
