/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <inttypes.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "base64.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "proto/telemetry/telemetry.pb-c.h"
#include "tef/lora.h"
#include "time.h"

float internal_temperature = 0;

const char *kLogTag = "main";

void createRecord(
  Telemetry__Record *record, Telemetry__Resource resource, double value,
  time_t time) {
  record->resource = resource;
  record->value = value;
  record->time = time;
}
std::string payload_packet = "";

void sendPacket() {
  std::vector<Telemetry__Record *> records;

  Telemetry__Record mcu_temperature_record = TELEMETRY__RECORD__INIT;
  ESP_LOGI(kLogTag, "Internal temperature record added");
  createRecord(
    &mcu_temperature_record, TELEMETRY__RESOURCE__RESOURCE_INTERNAL_TEMPERATURE,
    (double)internal_temperature, (time_t)1737680282);
  records.push_back(&mcu_temperature_record);

  Telemetry__TelemetryPack telemetry_pack = TELEMETRY__TELEMETRY_PACK__INIT;
  telemetry_pack.records = records.data();
  ESP_LOGI(kLogTag, "Records size: %d", records.size());
  telemetry_pack.n_records = records.size();
  std::vector<uint8_t> payload(
    telemetry__telemetry_pack__get_packed_size(&telemetry_pack));
  telemetry__telemetry_pack__pack(&telemetry_pack, payload.data());
  ESP_LOGI(kLogTag, "Packet of size %dB to be sent in binary:", payload.size());
  ESP_LOG_BUFFER_HEXDUMP(
    kLogTag, payload.data(), payload.size(), ESP_LOG_DEBUG);

  std::string base = base64_encode(
    reinterpret_cast<unsigned char const *>(payload.data()), payload.size(),
    false);
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
  if (lost != 0) {
    ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
  }
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

#if CONFIG_SENDER
void task_tx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  while (1) {
    TickType_t nowTick = xTaskGetTickCount();
    if (internal_temperature > 0) sendPacket();

    vTaskDelay(pdMS_TO_TICKS(60000));
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
  xTaskCreate(&sensorsTask, "Sensors", 1024 * 3, NULL, 5, NULL);
  vTaskDelay(pdMS_TO_TICKS(1000));
#if CONFIG_SENDER
  xTaskCreate(&task_tx, "TX", 1024 * 3, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}
