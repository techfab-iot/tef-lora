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
#include "lora.h"

#if CONFIG_SENDER
void task_tx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    TickType_t nowTick = xTaskGetTickCount();
    int send_len = sprintf((char *)buf, "Hello World!! %" PRIu32, nowTick);
    lora::sendPacket(buf, send_len);
    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
    int lost = lora::packetLost();
    if (lost != 0) {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }  // end while
}
#endif  // CONFIG_SENDER

#if CONFIG_RECEIVER
void task_rx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    lora::receive();  // put into receive mode
    if (lora::received()) {
      int rxLen = lora::receivePacket(buf, sizeof(buf));
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen,
        buf);
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }  // end while
}
#endif  // CONFIG_RECEIVER

extern "C" void app_main() {
  if (lora::init() == 0) {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

#if CONFIG_433MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
  lora::setFrequency(433e6);  // 433MHz
#elif CONFIG_866MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 866MHz");
  lora::setFrequency(866e6);  // 866MHz
#elif CONFIG_915MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
  lora::setFrequency(915e6);  // 915MHz
#elif CONFIG_OTHER
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
  long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
  lora::setFrequency(frequency);
#endif

  lora::enableCrc();

  int cr = 1;
  int bw = 7;
  int sf = 7;
#if CONFIG_ADVANCED
  cr = CONFIG_CODING_RATE;
  bw = CONFIG_BANDWIDTH;
  sf = CONFIG_SF_RATE;
#endif

  lora::setCodingRate(cr);
  // lora::setCodingRate(CONFIG_CODING_RATE);
  // cr = lora::getCodingRate();
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

  lora::setBandwidth(bw);
  // lora::setBandwidth(CONFIG_BANDWIDTH);
  // int bw = lora::getBandwidth();
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

  lora::setSpreadingFactor(sf);
  // lora::setSpreadingFactor(CONFIG_SF_RATE);
  // int sf = lora::getSpreadingFactor();
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

#if CONFIG_SENDER
  xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}
