/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "app/modules/mesh_node.h"
#include "app/modules/sensors.h"
#include "app/utils.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "tef/lora.h"
#include "tef/mesh.h"
namespace sensor = ::app::modules::sensors;

static constexpr gpio_num_t kGpioReset = GPIO_NUM_21;
static constexpr gpio_num_t kGpioCs = GPIO_NUM_18;  // NSS
static constexpr gpio_num_t kGpioSck = GPIO_NUM_17;
static constexpr gpio_num_t kGpioMiso = GPIO_NUM_15;
static constexpr gpio_num_t kGpioMosi = GPIO_NUM_16;
static constexpr double kVoltageMultiplier = 431993;
static constexpr double kCurrentMultiplier = 11135;
static constexpr double kActivePowerMultiplier = 9626328;

#if CONFIG_RECEIVER
void task_rx(void* pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1276/77/78/79 is 255
  std::string payload;
  printf(
    "log:\r\nVoltage(V),\tActive Power(W),\tPower "
    "Factor(%%),\tBLVoltage(V),\tBLCurrent(A),\tBLActive "
    "Power(W),\tBLPower Factor(%%)\tTimestamp(s),");
  while (1) {
    tef::lora::sx1276::receive();  // put into receive mode
    if (tef::lora::sx1276::received()) {
      int rxLen = tef::lora::sx1276::receivePacket(buf, sizeof(buf));
      payload = std::string((char*)buf, rxLen);
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen,
        buf);
      if (
        payload.size() > 3 && payload[0] == 'l' && payload[1] == 'o' &&
        payload[2] == 'g' && payload[3] == ':') {
        payload += "\t" + std::to_string(app::utils::getTimestamp()) + ",";
        printf("%s", payload.substr(4).c_str());
      }
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }  // end while
}
#endif  // CONFIG_RECEIVER

void wifi_event_handler(
  void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (
    event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ESP_LOGI("wifi_event_handler", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
  }
}
void wifi_init_sta(const char* ssid, const char* password) {
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  esp_event_handler_instance_register(
    WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
  esp_event_handler_instance_register(
    IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

  wifi_config_t wifi_config = {};
  strcpy((char*)wifi_config.sta.ssid, ssid);
  strcpy((char*)wifi_config.sta.password, password);

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();
}

extern "C" void app_main() {
  esp_log_level_set("*", ESP_LOG_ERROR);
  tef::lora::sx1276::setPins(
    kGpioReset, kGpioCs, kGpioSck, kGpioMiso, kGpioMosi);
  if (tef::lora::sx1276::init() == 0) {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }
  tef::lora::sx1276::setFrequency(915e6);  // 915MHz
  tef::lora::sx1276::enableCrc();
  tef::lora::sx1276::setCodingRate(1);
  tef::lora::sx1276::setBandwidth(7);
  tef::lora::sx1276::setSpreadingFactor(7);

  tef::mesh::addMessageResponseAction(
    "telemetry", "telemetry_ack", app::modules::sensors::telemetryCallback);
  app::modules::mesh_node::init();

#if CONFIG_SENDER
  sensor::setVoltageMultiplier(kVoltageMultiplier);
  sensor::setCurrentMultiplier(kCurrentMultiplier);
  sensor::setActivePowerMultiplier(kActivePowerMultiplier);
  sensor::init();
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 3, NULL, 5, NULL);
#endif
}
