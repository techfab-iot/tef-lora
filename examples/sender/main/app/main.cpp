/**
 * LoRa sender example using the aggregated tef/lora API.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tef/lora.h"
#include "tef/boards/meshlink_gateway.h"

namespace board = tef::boards::meshlink_gateway::v0_5_0;
namespace radio = tef::lora::sx1262;

static constexpr const char *kLogTag = "main";

namespace {

constexpr char kNvsNamespace[] = "meshlink";
constexpr char kBootCountKey[] = "bootCnt";

uint32_t s_boot_count = 0;

bool formatMacAddress(char *out, size_t out_size) {
  uint8_t mac[6];
  if (esp_efuse_mac_get_default(mac) != ESP_OK) {
    ESP_LOGE(kLogTag, "Failed to read MAC address");
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
    ESP_LOGW(kLogTag, "NVS init failed: %s", esp_err_to_name(err));
    s_boot_count = 1;
    return s_boot_count;
  }

  nvs_handle_t handle = 0;
  err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    ESP_LOGW(kLogTag, "NVS open failed: %s", esp_err_to_name(err));
    s_boot_count = 1;
    return s_boot_count;
  }

  uint32_t count = 0;
  err = nvs_get_u32(handle, kBootCountKey, &count);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(kLogTag, "NVS read bootCnt failed: %s", esp_err_to_name(err));
    count = 0;
  }

  count++;
  err = nvs_set_u32(handle, kBootCountKey, count);
  if (err == ESP_OK) {
    err = nvs_commit(handle);
  }
  nvs_close(handle);

  if (err != ESP_OK) {
    ESP_LOGW(kLogTag, "Boot count write failed: %s", esp_err_to_name(err));
  }

  s_boot_count = count;
  return s_boot_count;
}

}  // namespace

void task_tx(void *pvParameters) {
  (void)pvParameters;
  ESP_LOGI(pcTaskGetName(NULL), "Start");

  char mac_str[18] = {0};
  if (!formatMacAddress(mac_str, sizeof(mac_str))) {
    strlcpy(mac_str, "00-00-00-00-00-00", sizeof(mac_str));
  }

  uint32_t message_count = 0;
  uint8_t buf[256];  // Maximum Payload size of SX1261/62/68 is 255

  while (1) {
    const uint32_t message_number = ++message_count;
    const int tx_len = snprintf(
      (char *)buf, sizeof(buf),
      "hello from %s, boot #%" PRIu32 ", message=%" PRIu32, mac_str,
      s_boot_count, message_number);
    if (tx_len < 0 || static_cast<size_t>(tx_len) >= sizeof(buf)) {
      ESP_LOGE(pcTaskGetName(NULL), "Message formatting failed");
      vTaskDelay(pdMS_TO_TICKS(10000));
      continue;
    }

    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", tx_len);
    if (radio::send(buf, tx_len, tef::lora::TxMode::kSync) == false) {
      ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");
    }

    int lost = radio::getPacketLost();
    if (lost != 0) {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

extern "C" void app_main() {
  s_boot_count = initBootCount();

  // Initialize LoRa
  constexpr auto pins = board::kLoraRadioPins;
  radio::init(
    pins.rst, pins.nss, pins.sck, pins.miso, pins.mosi, pins.busy,
    pins.dio1, pins.txen, pins.rxen);
  radio::debugPrint(false);
  int8_t txPowerInDbm = 22;

  // LoRa PHY parameters are fixed here so sender and receiver examples always
  // agree, instead of being independently configurable per-firmware via
  // Kconfig. The chip-specific driver stays behind `radio`.
  uint32_t frequencyInHz = 915000000;
  ESP_LOGI(kLogTag, "Frequency is 915MHz");

  ESP_LOGW(kLogTag, "Enable TCXO %.1fV", (double)board::kLoraTcxoVoltage);
  float tcxoVoltage = board::kLoraUseTcxo ? board::kLoraTcxoVoltage : 0.0f;
  bool useRegulatorLDO = board::kLoraUseRegulatorLdo;

  if (
    radio::begin(frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) !=
    0) {
    ESP_LOGE(kLogTag, "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

  constexpr uint8_t spreadingFactor = 7;
  constexpr auto bandwidth = tef::lora::Bandwidth::k125KHz;
  constexpr auto codingRate = tef::lora::CodingRate::k4_5;
  constexpr uint16_t preambleLength = 8;
  constexpr uint8_t payloadLen = 0;
  constexpr bool crcOn = true;
  constexpr bool invertIrq = false;
  radio::config(
    spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn,
    invertIrq);

  xTaskCreate(&task_tx, "TX", 1024 * 4, NULL, 5, NULL);
}
