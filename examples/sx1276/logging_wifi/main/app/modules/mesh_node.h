#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace app::modules::mesh_node {
void attachSingleLevelLed(
  gpio_num_t mesh_gpio, SemaphoreHandle_t mutex, bool* ota_running);
void init();
}  // namespace app::modules::mesh_node