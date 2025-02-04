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
#include "sx1262.h"

static const char *TAG = "MAIN";

static constexpr gpio_num_t kGpioReset = GPIO_NUM_5;
static constexpr gpio_num_t kGpioCs = GPIO_NUM_8;  // NSS
static constexpr gpio_num_t kGpioSck = GPIO_NUM_16;
static constexpr gpio_num_t kGpioMiso = GPIO_NUM_17;
static constexpr gpio_num_t kGpioMosi = GPIO_NUM_18;
static constexpr gpio_num_t kGpioBusy = GPIO_NUM_15;
static constexpr gpio_num_t kGpioTxen = GPIO_NUM_6;
static constexpr gpio_num_t kGpioRxen = GPIO_NUM_NC;

#if CONFIG_SENDER
void task_tx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1261/62/68 is 255
  while (1) {
    TickType_t nowTick = xTaskGetTickCount();
    int txLen = sprintf((char *)buf, "Hello World %" PRIu32, nowTick);
    ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", txLen);

    // Wait for transmission to complete
    if (LoRaSend(buf, txLen, SX126x_TXMODE_SYNC) == false) {
      ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");
    }

    // Do not wait for the transmission to be completed
    // LoRaSend(buf, txLen, SX126x_TXMODE_ASYNC );

    int lost = GetPacketLost();
    if (lost != 0) {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }  // end while
}
#endif  // CONFIG_SENDER

#if CONFIG_RECEIVER
void task_rx(void *pvParameters) {
  ESP_LOGI(pcTaskGetName(NULL), "Start");
  uint8_t buf[256];  // Maximum Payload size of SX1261/62/68 is 255
  while (1) {
    uint8_t rxLen = tef::lora::sx1262::receive(buf, sizeof(buf));
    if (rxLen > 0) {
      ESP_LOGI(
        pcTaskGetName(NULL), "%d byte packet received:[%.*s]", rxLen, rxLen,
        buf);

      int8_t rssi, snr;
      tef::lora::sx1262::getPacketStatus(&rssi, &snr);
      ESP_LOGI(pcTaskGetName(NULL), "rssi=%d[dBm] snr=%d[dB]", rssi, snr);
    }
    vTaskDelay(1);  // Avoid WatchDog alerts
  }  // end while
}
#endif  // CONFIG_RECEIVER

extern "C" void app_main() {
  // Initialize LoRa
  tef::lora::sx1262::init(
    kGpioReset, kGpioCs, kGpioSck, kGpioMiso, kGpioMosi, kGpioBusy, kGpioTxen,
    kGpioRxen);
  int8_t txPowerInDbm = 22;

  uint32_t frequencyInHz = 0;
#if CONFIG_433MHZ
  frequencyInHz = 433000000;
  ESP_LOGI(TAG, "Frequency is 433MHz");
#elif CONFIG_866MHZ
  frequencyInHz = 866000000;
  ESP_LOGI(TAG, "Frequency is 866MHz");
#elif CONFIG_915MHZ
  frequencyInHz = 915000000;
  ESP_LOGI(TAG, "Frequency is 915MHz");
#elif CONFIG_OTHER
  ESP_LOGI(TAG, "Frequency is %dMHz", CONFIG_OTHER_FREQUENCY);
  frequencyInHz = CONFIG_OTHER_FREQUENCY * 1000000;
#endif

#if CONFIG_USE_TCXO
  ESP_LOGW(TAG, "Enable TCXO");
  float tcxoVoltage = 3.3;      // use TCXO
  bool useRegulatorLDO = true;  // use DCDC + LDO
#else
  ESP_LOGW(TAG, "Disable TCXO");
  float tcxoVoltage = 0.0;       // don't use TCXO
  bool useRegulatorLDO = false;  // use only LDO in all modes
#endif

  // LoRaDebugPrint(true);
  if (
    tef::lora::sx1262::begin(
      frequencyInHz, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0) {
    ESP_LOGE(TAG, "Does not recognize the module");
    while (1) {
      vTaskDelay(1);
    }
  }

  uint8_t spreadingFactor = 7;
  uint8_t bandwidth = 4;
  uint8_t codingRate = 1;
  uint16_t preambleLength = 8;
  uint8_t payloadLen = 0;
  bool crcOn = true;
  bool invertIrq = false;
#if CONFIG_ADVANCED
  spreadingFactor = CONFIG_SF_RATE;
  bandwidth = CONFIG_BANDWIDTH;
  codingRate = CONFIG_CODING_RATE;
#endif
  tef::lora::sx1262::config(
    spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn,
    invertIrq);

#if CONFIG_SENDER
  xTaskCreate(&task_tx, "TX", 1024 * 4, NULL, 5, NULL);
#endif
#if CONFIG_RECEIVER
  xTaskCreate(&task_rx, "RX", 1024 * 4, NULL, 5, NULL);
#endif
}
