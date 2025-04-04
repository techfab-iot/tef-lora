#include "app/modules/mesh_node.h"

#include <string.h>

#include <vector>

#include "../utils.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "tef/mesh.h"
#include "tef/nvs.h"
#include "tef/wifi-bridge.h"

namespace app::modules::mesh_node {
static constexpr auto kLogTag = "mesh_module";
std::vector<uint8_t> mqtt_network_buffer(8480U);

void attachSingleLevelLed(
  gpio_num_t mesh_gpio, SemaphoreHandle_t mutex, bool *ota_running) {
  tef::mesh::attachSingleLevelLed(mesh_gpio, mutex, ota_running);
}

void init() {
  tef::nvs::Namespace("mesh").init();
  // esp_wifi_clear_ap_list();
  tef::wifi_bridge::BridgeConfig_t bridge_config = {
    .interface = tef::wifi_bridge::kWifiInterface,
    .sta =
      {
        .ssid = "TF-IoT",
        .password = "&rrGSr5f5W@&tR",
        .uses_lr_protocol = false,
      },
    .ap =
      {
        .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
        .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
        .uses_lr_protocol = false,
      },
    .thing_id = "2a1266cd-3cf6-44f8-a99f-6ad1ec7a2134",
    .is_hidden = false,
  };
  tef::wifi_bridge::init(bridge_config);

  tef::mesh::MeshConfig_t mesh_config = {
    .mesh_id = 77,
    .enable_lan_ota = true,
  };
  tef::mesh::init(mesh_config);

  esp_sntp_config_t ntpConfig = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
  esp_netif_sntp_init(&ntpConfig);
  auto ntp_result = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
  while (ntp_result != ESP_OK) {
    ESP_LOGE(kLogTag, "Failed to update system time within 10s timeout");
    ntp_result = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
  }
  ESP_LOGI(
    kLogTag, "Time synchronized successfully, Timestamp: %" PRIu64,
    utils::getTimestamp());
  while (tef::mesh::getMeshLevel() == 0) {
    ESP_LOGI(kLogTag, "Waiting for mesh level...");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGI(kLogTag, "Mesh level: %d", tef::mesh::getMeshLevel());
  vTaskDelay(pdMS_TO_TICKS(5000));
}
}  // namespace app::modules::mesh_node