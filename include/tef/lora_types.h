#pragma once

#include <stdint.h>

namespace tef::lora {

enum class Bandwidth : uint8_t {
  k7_8KHz = 0,
  k10_4KHz = 1,
  k15_6KHz = 2,
  k20_8KHz = 3,
  k31_25KHz = 4,
  k41_7KHz = 5,
  k62_5KHz = 6,
  k125KHz = 7,
  k250KHz = 8,
  k500KHz = 9,
};

enum class CodingRate : uint8_t {
  k4_5 = 1,
  k4_6 = 2,
  k4_7 = 3,
  k4_8 = 4,
};

enum class TxMode : uint8_t {
  kAsync = 0x01,
  kSync = 0x02,
  kBackToRx = 0x04,
};

}  // namespace tef::lora
