/*
 * Turn-taking example using the aggregated tef/lora API.
 *
 * Sends a packet, listens for a short window, then waits a randomized
 * backoff before sending again.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app/utils.h"
#include "tef/lora.h"
// #include "tef/boards/testbed.h"
#include "tef/boards/meshlink_gateway.h"

// namespace board = tef::boards::testbed::v0_3_2;
namespace board = tef::boards::meshlink_gateway::v0_5_0;
// namespace radio = tef::lora::sx1276;
namespace radio = tef::lora::sx1262;

static constexpr auto kLogTag = "main";

namespace {

constexpr uint32_t kListenWindowMs = 30000;
constexpr uint32_t kBackoffMinMs = 750;
constexpr uint32_t kBackoffMaxMs = 2500;

uint32_t s_boot_count = 0;

template <typename Pins>
void initRadio(const Pins &pins) {
  if constexpr (std::is_same_v<Pins, tef::boards::Sx127xRadioPins>) {
    radio::init(
      pins.rst, pins.nss, pins.sck, pins.miso, pins.mosi, GPIO_NUM_NC,
      pins.dio[0], GPIO_NUM_NC, GPIO_NUM_NC);
  } else {
    radio::init(
      pins.rst, pins.nss, pins.sck, pins.miso, pins.mosi, pins.busy,
      pins.dio1, pins.txen, pins.rxen);
  }
}

template <typename Pins>
void configureRadio(const Pins &) {
  constexpr uint32_t frequencyInHz = 915000000;
  ESP_LOGI(kLogTag, "Frequency is 915MHz");

  if constexpr (std::is_same_v<Pins, tef::boards::Sx127xRadioPins>) {
    constexpr int8_t txPowerInDbm = 17;
    if (radio::begin(frequencyInHz, txPowerInDbm, 0.0f, false) != 0) {
      ESP_LOGE(kLogTag, "Does not recognize the module");
      while (1) {
        vTaskDelay(1);
      }
    }
  } else {
    constexpr int8_t txPowerInDbm = 22;
    ESP_LOGW(
      kLogTag, "Enable TCXO %.1fV",
      (double)board::kLoraTcxoVoltage);
    const float tcxoVoltage =
      board::kLoraUseTcxo ? board::kLoraTcxoVoltage : 0.0f;
    const bool useRegulatorLDO = board::kLoraUseRegulatorLdo;
    if (
      radio::begin(
        frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
      ESP_LOGE(kLogTag, "Does not recognize the module");
      while (1) {
        vTaskDelay(1);
      }
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
}

void logReceivedPacket(const uint8_t *buf, uint8_t len) {
  int8_t rssi = 0;
  int8_t snr = 0;
  radio::getPacketStatus(&rssi, &snr);
  ESP_LOGI(
    pcTaskGetName(NULL), "%u byte packet received:[%.*s] rssi=%d snr=%d",
    static_cast<unsigned>(len), len, buf, rssi, snr);
}

}  // namespace

void task_turntaking(void *pvParameters) {
  (void)pvParameters;
  ESP_LOGI(pcTaskGetName(NULL), "Start");

  char mac_str[18] = {0};
  if (!app::utils::formatMacAddress(mac_str, sizeof(mac_str))) {
    strlcpy(mac_str, "00-00-00-00-00-00", sizeof(mac_str));
  }

  uint32_t message_count = 0;
  uint8_t buf[256];  // Maximum Payload size of is 255 Bytes

  while (1) {
    const uint32_t message_number = ++message_count;
    const int tx_len = snprintf(
      (char *)buf, sizeof(buf),
      "hello from %s, boot #%" PRIu32 ", message=%" PRIu32, mac_str,
      s_boot_count, message_number);
    if (tx_len < 0 || static_cast<size_t>(tx_len) >= sizeof(buf)) {
      ESP_LOGE(pcTaskGetName(NULL), "Message formatting failed");
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", tx_len);
    if (!radio::send(buf, tx_len, tef::lora::TxMode::kSync)) {
      ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");
    }

    const TickType_t listen_ticks = pdMS_TO_TICKS(kListenWindowMs);
    const TickType_t listen_start = xTaskGetTickCount();
    ESP_LOGI(
      pcTaskGetName(NULL), "RX window start (%" PRIu32 " ms)",
      static_cast<uint32_t>(kListenWindowMs));

    while ((xTaskGetTickCount() - listen_start) < listen_ticks) {
      const uint8_t rx_len = radio::receive(buf, sizeof(buf));
      if (rx_len > 0) {
        logReceivedPacket(buf, rx_len);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    const uint32_t backoff_ms =
      app::utils::randomDelayMs(kBackoffMinMs, kBackoffMaxMs);
    ESP_LOGI(
      pcTaskGetName(NULL), "Backoff=%" PRIu32 " ms before next TX", backoff_ms);
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
  }
}

extern "C" void app_main() {
  s_boot_count = app::utils::initBootCount();

  constexpr auto pins = board::kLoraRadioPins;
  initRadio(pins);
  radio::debugPrint(false);
  configureRadio(pins);

  xTaskCreate(&task_turntaking, "TURN", 1024 * 4, NULL, 5, NULL);
}
