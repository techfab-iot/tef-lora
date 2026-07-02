/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tef/boards/testbed.h"
#include "tef/lora.h"

namespace board = tef::boards::testbed::v0_3_2;

#if CONFIG_SENDER
void task_tx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    TickType_t nowTick = xTaskGetTickCount();
    int send_len = sprintf((char *)buf, "Hello World!! %" PRIu32, nowTick);
    tef::lora::sx1276::sendPacket(buf, send_len);
    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
    int lost = tef::lora::sx1276::packetLost();
    if (lost != 0) {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }  // end while
}
#endif  // CONFIG_SENDER

#if CONFIG_RECEIVER
void task_rx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    tef::lora::sx1276::receive();  // put into receive mode
    if (tef::lora::sx1276::received()) {
      int rxLen = tef::lora::sx1276::receivePacket(buf, sizeof(buf));
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen,
        buf);
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }  // end while
}
#endif  // CONFIG_RECEIVER

extern "C" void app_main() {
  constexpr auto pins = board::kLoraRadioPins;
  tef::lora::sx1276::setPins(
    pins.rst, pins.nss, pins.sck, pins.miso, pins.mosi);
  if (tef::lora::sx1276::init() == 0) {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

  // LoRa PHY parameters: frequency (915MHz), coding rate (4/5), bandwidth
  // (125kHz) and spreading factor (7) are fixed here so sender and receiver
  // examples always agree, instead of being independently configurable
  // per-firmware via Kconfig.
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
  tef::lora::sx1276::setFrequency(915e6);  // 915MHz

  tef::lora::sx1276::enableCrc();

  int cr = 1;  // 4/5
  int bw = 7;  // 125kHz
  int sf = 7;

  tef::lora::sx1276::setCodingRate(cr);
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

  tef::lora::sx1276::setBandwidth(bw);
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

  tef::lora::sx1276::setSpreadingFactor(sf);
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

#if CONFIG_SENDER
  xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}
