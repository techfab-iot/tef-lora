#include "app/utils.h"

#include <cstdio>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace app::utils {

namespace {

constexpr char kTag[] = "main";
constexpr char kNvsNamespace[] = "meshlink";
constexpr char kBootCountKey[] = "bootCnt";

}  // namespace

uint32_t randomDelayMs(uint32_t min_ms, uint32_t max_ms) {
  if (max_ms <= min_ms) {
    return min_ms;
  }
  return min_ms + (esp_random() % (max_ms - min_ms + 1));
}

bool formatMacAddress(char *out, size_t out_size) {
  uint8_t mac[6];
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    ESP_LOGE(kTag, "Failed to read MAC address");
    return false;
  }

  const int written = snprintf(
    out, out_size, "%02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2],
    mac[3], mac[4], mac[5]);
  return written > 0 && static_cast<size_t>(written) < out_size;
}

uint32_t initBootCount() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGW(kTag, "NVS init failed: %s", esp_err_to_name(err));
    return 1;
  }

  nvs_handle_t handle = 0;
  err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(kTag, "NVS open failed: %s", esp_err_to_name(err));
    return 1;
  }

  uint32_t count = 0;
  err = nvs_get_u32(handle, kBootCountKey, &count);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kTag, "NVS read bootCnt failed: %s", esp_err_to_name(err));
    count = 0;
  }

  count++;
  err = nvs_set_u32(handle, kBootCountKey, count);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);

  if (err != ESP_OK) {
    ESP_LOGW(kTag, "Boot count write failed: %s", esp_err_to_name(err));
  }

  return count;
}

}  // namespace app::utils
