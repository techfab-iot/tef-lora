#include "app/utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace app::utils {
void abortWithError(
  std::string_view tag, std::string_view message, esp_err_t error) {
  ESP_LOGE(tag.data(), "%s: %s", message.data(), esp_err_to_name(error));
  abort();
}

time_t getTimestamp() {
  time_t timestamp;
  time(&timestamp);

  return timestamp;
}

void wait(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

}  // namespace app::utils
