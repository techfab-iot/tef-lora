#include <inttypes.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "proto/telemetry/telemetry.pb-c.h"
#include "tef/lora.h"
#include "tef/proto.h"
#include "time.h"
#include "utils/base64.h"

float internal_temperature = 0;

const char *kLogTag = "main";

std::string payload_packet = "";

void sendPacket() {
  tef::proto::addRecord(
    TELEMETRY__RESOURCE__RESOURCE_INTERNAL_TEMPERATURE, internal_temperature);
  ESP_LOGI(kLogTag, "Internal temperature record added");

  std::vector<uint8_t> payload = tef::proto::createPack();
  auto base = tef::proto::encodePack(payload);
  if (base.empty()) {
    ESP_LOGW(kLogTag, "Packet not sent, it's empty!");
    return;
  }
  payload_packet.clear();
  payload_packet += base;
  ESP_LOGI(kLogTag, "Packet to be sent: %s", payload_packet.c_str());
  tef::lora::sendPacket(
    reinterpret_cast<uint8_t *>(payload_packet.data()), payload_packet.size());
  int lost = tef::lora::packetLost();
  if (lost != 0) ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);

  tef::proto::clearRecords();
}

void sensorsTask(void *arg) {
  ESP_LOGI(pcTaskGetName(NULL), "Sensors Task is Running.");
  temperature_sensor_handle_t temp_sensor = NULL;
  temperature_sensor_config_t temp_sensor_config =
    TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  ESP_ERROR_CHECK(
    temperature_sensor_install(&temp_sensor_config, &temp_sensor));
  ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));
  ESP_LOGI(
    pcTaskGetName(NULL), "Microcontroller Internal Temperature Sensor Enabled");

  while (true) {
    ESP_ERROR_CHECK(
      temperature_sensor_get_celsius(temp_sensor, &internal_temperature));
    ESP_LOGI(
      pcTaskGetName(NULL), "Microcontroller temperature = %.02f ºC",
      internal_temperature);
    vTaskDelay(pdMS_TO_TICKS(10 * 1000));
  }
  vTaskDelete(NULL);
}

void txTask(void *pvParameters) {
  ESP_LOGI(kLogTag, "Start");
  while (1) {
    if (internal_temperature > 0) sendPacket();
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

extern "C" void app_main() {
  if (tef::lora::init() == 0) {
    ESP_LOGE(kLogTag, "Does not recognize the module... wrong gpio pins?");
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
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

  tef::lora::setBandwidth(bw);
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

  tef::lora::setSpreadingFactor(sf);
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

  xTaskCreate(&sensorsTask, "Sensors", 1024 * 3, NULL, 5, NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));
  xTaskCreate(&txTask, "TX", 1024 * 3, NULL, 5, NULL);
}
