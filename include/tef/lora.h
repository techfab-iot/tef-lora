#pragma once
#include <string>

#include "driver/gpio.h"
#include "hal/spi_types.h"
#include "sx1262.h"
#include "sx1276.h"
namespace tef::lora {

typedef enum {
  SX1262,
  SX1276,
} device_t;
}  // namespace tef::lora
