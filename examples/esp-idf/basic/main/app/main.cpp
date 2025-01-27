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
#include "tef/lora.h"

static constexpr gpio_num_t kGpioReset = GPIO_NUM_18;
static constexpr gpio_num_t kGpioCs = GPIO_NUM_17;  // NSS
static constexpr gpio_num_t kGpioSck = GPIO_NUM_16;
static constexpr gpio_num_t kGpioMiso = GPIO_NUM_7;
static constexpr gpio_num_t kGpioMosi = GPIO_NUM_15;

#if CONFIG_SENDER
void task_tx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    TickType_t nowTick = xTaskGetTickCount();
    int send_len = sprintf((char *)buf, "Hello World!! %" PRIu32, nowTick);
    tef::lora::sendPacket(buf, send_len);
    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
    int lost = tef::lora::packetLost();
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
    tef::lora::receive();  // put into receive mode
    if (tef::lora::received()) {
      int rxLen = tef::lora::receivePacket(buf, sizeof(buf));
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen,
        buf);
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }  // end while
}
#endif  // CONFIG_RECEIVER

extern "C" void app_main() {
  tef::lora::setPins(kGpioReset, kGpioCs, kGpioSck, kGpioMiso, kGpioMosi);
  if (tef::lora::init() == 0) {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

#if CONFIG_433MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 433MHz");
  tef::lora::setFrequency(433e6);  // 433MHz
#elif CONFIG_866MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 866MHz");
  tef::lora::setFrequency(866e6);  // 866MHz
#elif CONFIG_915MHZ
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
  tef::lora::setFrequency(915e6);  // 915MHz
#elif CONFIG_OTHER
  ESP_LOGI(pcTaskGetName(NULL), "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
  long frequency = CONFIG_OTHER_FREQUENCY * 1000000;
  tef::lora::setFrequency(frequency);
#endif

  tef::lora::enableCrc();

  int cr = 1;
  int bw = 7;
  int sf = 7;
#if CONFIG_ADVANCED
  cr = CONFIG_CODING_RATE;
  bw = CONFIG_BANDWIDTH;
  sf = CONFIG_SF_RATE;
#endif

  tef::lora::setCodingRate(cr);
  // tef::lora::setCodingRate(CONFIG_CODING_RATE);
  // cr = lora::getCodingRate();
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

  tef::lora::setBandwidth(bw);
  // tef::lora::setBandwidth(CONFIG_BANDWIDTH);
  // int bw = lora::getBandwidth();
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

  tef::lora::setSpreadingFactor(sf);
  // tef::lora::setSpreadingFactor(CONFIG_SF_RATE);
  // int sf = lora::getSpreadingFactor();
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

#if CONFIG_SENDER
  xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}
