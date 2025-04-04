#pragma once

#include <ctime>
#include <string_view>

#include "esp_err.h"
#include "esp_log.h"

namespace app::utils {
void abortWithError(
  std::string_view tag, std::string_view message, esp_err_t error);

time_t getTimestamp();

void wait(uint32_t ms);

}  // namespace app::utils
