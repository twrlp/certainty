#include "module_state.h"

namespace certainty {

ModuleState g_module = {};

TwoWire &g_i2cBus = Wire;

}  // namespace certainty
