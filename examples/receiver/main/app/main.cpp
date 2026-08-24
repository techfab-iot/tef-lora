/**
 * LoRa receiver example using the aggregated tef/lora API.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tef/lora.h"
#include "tef/boards/testbed.h"

namespace board = tef::boards::testbed::v0_3_2;
namespace radio = tef::lora::sx1276;

static constexpr const char *kLogTag = "main";

void task_rx(void *pvParameters) {
  (void)pvParameters;
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    radio::receive();  // put into receive mode
    if (radio::received()) {
      int rxLen = radio::receivePacket(buf, sizeof(buf));
      int rssi = radio::packetRssi();
      float snr = radio::packetSnr();
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s] rssi=%d snr=%.2f",
        rxLen, rxLen, buf, rssi, snr);
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }
}

extern "C" void app_main() {
  constexpr auto pins = board::kLoraRadioPins;
  if (
    radio::init(
      pins.rst, pins.nss, pins.sck, pins.miso, pins.mosi, GPIO_NUM_NC,
      pins.dio[1], GPIO_NUM_NC, GPIO_NUM_NC) == 0) {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

  // LoRa PHY parameters are fixed here so sender and receiver examples always
  // agree, instead of being independently configurable per-firmware via
  // Kconfig. The chip-specific driver stays behind `radio`.
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
  radio::setFrequency(915e6);

  radio::enableCrc();

  constexpr auto codingRate = tef::lora::CodingRate::k4_5;
  constexpr auto bandwidth = tef::lora::Bandwidth::k125KHz;
  constexpr int sf = 7;

  radio::setCodingRate(codingRate);
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", static_cast<int>(codingRate));

  radio::setBandwidth(bandwidth);
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", static_cast<int>(bandwidth));

  radio::setSpreadingFactor(sf);
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
}
