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
  tef::lora::sx1276::sendPacket(
    reinterpret_cast<uint8_t *>(payload_packet.data()), payload_packet.size());
  int lost = tef::lora::sx1276::packetLost();
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
  if (tef::lora::sx1276::init() == 0) {
    ESP_LOGE(kLogTag, "Does not recognize the module... wrong gpio pins?");
    while (1) {
      vTaskDelay(1);
    }
  }

  // LoRa PHY parameters: frequency (915MHz), coding rate (4/5), bandwidth
  // (125kHz) and spreading factor (7) are fixed here so sender and receiver
  // examples always agree, instead of being independently configurable
  // per-firmware via Kconfig. Matches examples/sx1276/basic.
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

  xTaskCreate(&sensorsTask, "Sensors", 1024 * 3, NULL, 5, NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));
  xTaskCreate(&txTask, "TX", 1024 * 3, NULL, 5, NULL);
}
