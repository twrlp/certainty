#include "rng.h"
#include "pico/time.h"

namespace certainty {

static uint32_t rngState = 0xA341316Cu;

static inline uint32_t nextRng() {
  rngState ^= rngState << 13u;
  rngState ^= rngState >> 17u;
  rngState ^= rngState << 5u;
  return rngState;
}

bool sampleLoopRetrigger(uint8_t probPercent) {
  if (probPercent >= 100) return true;
  if (probPercent == 0) return false;
  rngState ^= (uint32_t)time_us_64();
  nextRng();
  return (rngState % 100u) < probPercent;
}

float randFloat01() {
  return float(nextRng() >> 8) / 16777216.0f;
}

float gaussianWhiteNoise() {
  // CLT approximation: sum of 4 uniform [0,1) → mean=2, var=4*(1/12)=1/3
  // Subtract mean and scale by 1/sqrt(1/3) = sqrt(3) ≈ 1.7321 → std_dev ≈ 1
  const float sum = randFloat01() + randFloat01() + randFloat01() + randFloat01();
  return (sum - 2.0f) * 1.7321f;
}

float pinkNoiseSample(PinkNoiseState &state) {
  state.counter++;
  // Determine which row to update based on trailing zeros of counter.
  // Row 0 updates every call, row 1 every 2nd, row 2 every 4th, etc.
  uint32_t c = state.counter;
  uint8_t row = 0;
  while (row < PINK_NOISE_ROWS - 1 && (c & 1u) == 0) {
    row++;
    c >>= 1;
  }
  // Replace the selected row with a new random value in [-1, 1]
  const float newVal = randFloat01() * 2.0f - 1.0f;
  state.runningSum -= state.rows[row];
  state.rows[row] = newVal;
  state.runningSum += newVal;
  // Normalize: runningSum is sum of PINK_NOISE_ROWS values each in [-1,1].
  // Empirical std dev of the sum ≈ sqrt(PINK_NOISE_ROWS) * std_dev_uniform
  // std_dev_uniform([-1,1]) = 1/sqrt(3). So std_dev ≈ sqrt(8)/sqrt(3) ≈ 1.63.
  // Divide by 1.63 to get ~std_dev=1.
  return state.runningSum / 1.63f;
}

void initPinkNoise(PinkNoiseState &state) {
  state.counter = 0;
  state.runningSum = 0.0f;
  for (uint8_t i = 0; i < PINK_NOISE_ROWS; ++i) {
    state.rows[i] = randFloat01() * 2.0f - 1.0f;
    state.runningSum += state.rows[i];
  }
}

}  // namespace certainty
