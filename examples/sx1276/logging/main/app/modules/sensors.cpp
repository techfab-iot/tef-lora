#include "app/modules/sensors.h"

#include <math.h>

#include <array>
#include <string>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "tef/hlw8012.h"
#include "tef/lora.h"
namespace app::modules::sensors {
static constexpr auto *kLogTag = "sensors_module";
typedef struct Hlw8012Pins {
  gpio_num_t cf;
  gpio_num_t cf1;
  gpio_num_t sel;
  bool invert_sel_pin_state;
} Hlw8012Pins;

typedef struct Hlw8012Resistors {
  double current;
  double voltage_upstream;
  double voltage_downstream;
} Hlw8012Resistors;

typedef struct Hlw8012Params {
  Hlw8012Pins hlw8012_pins;
  Hlw8012Resistors hlw8012_resistors;
  bool use_interrupts;
  unsigned long pulse_timeout;
} Hlw8012Params;

static bool must_read_current = false;
static double voltage = 0;
static double current = 0;
static double active_power = 0;
static double apparent_power = 0;
static double power_factor = 0;

static double voltage_multiplier;
static double current_multiplier;
static double active_power_multiplier;

static tef::hlw8012::Hlw8012 *hlw8012;

static Hlw8012Params hlw8012_params;

static void setVoltage(double value) { voltage = value; }
double getVoltage() { return voltage; }

void updateCurrent() { must_read_current = true; }
static void setCurrent(double value) { current = value; }
double getCurrent() { return current; }

static void setActivePower(double value) { active_power = value; }
double getActivePower() { return active_power; }

static void setApparentPower(double value) { apparent_power = value; }
double getApparentPower() { return apparent_power; }

static void setPowerFactor(int value) { power_factor = value; }
int getPowerFactor() { return power_factor; }

void setVoltageMultiplier(double value) { voltage_multiplier = value; }
void setCurrentMultiplier(double value) { current_multiplier = value; }
void setActivePowerMultiplier(double value) { active_power_multiplier = value; }

void configureHlw8012(const Hlw8012Params &params) { hlw8012_params = params; }

void monitorAcTask(void *) {
  constexpr uint32_t kUpdateMeasurementIntervalMs = 1000;
  double read_value = 0;
  ESP_LOGD(kLogTag, "Initialized monitorAcTask");
  uint8_t is_truly_zero_counter = 0;
  uint8_t is_truly_not_zero_counter = 0;
  uint8_t count = 0;
  std::string log =
    "log:\r\nVoltage(V),\tActive Power(W),\tPower Factor(%),\tTimestamp(s)\r\n";
  while (true) {
    // VOLTAGE
    read_value = hlw8012->getVoltage();
    log += std::to_string(read_value) + ",\t";
    // CURRENT
    // read_value = hlw8012->getCurrent();
    // log += "Current: " + std::to_string(read_value) + "A" + "\r\n";
    // ACTIVE POWER
    read_value = hlw8012->getActivePower();
    log += std::to_string(read_value) + ",\t\t";
    // APPARENT POWER
    // read_value = hlw8012->getApparentPower();
    // log += std::to_string(read_value) + ",\t";
    // REACTIVE POWER
    // read_value = hlw8012->getReactivePower();
    // log += std::to_string(read_value) + ",\t";
    // POWER FACTOR
    read_value = hlw8012->getPowerFactor() * 100;
    log += std::to_string(read_value) + ",\t";
    if (count > 3)
      tef::lora::sx1276::sendPacket((uint8_t *)log.c_str(), log.length());
    else
      count++;
    ESP_LOGI(
      kLogTag, "Packet of %d bytes sent:\n%s", log.length(), log.c_str());
    int lost = tef::lora::sx1276::packetLost();
    if (lost != 0) {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }
    log = "log:\n";
    vTaskDelay(1);
  }

  ESP_LOGW(kLogTag, "Finished monitorAcTask");
  vTaskDelete(nullptr);
}

static void initHlw8012() {
  hlw8012 = new tef::hlw8012::Hlw8012(
    hlw8012_params.hlw8012_pins.cf, hlw8012_params.hlw8012_pins.cf1,
    hlw8012_params.hlw8012_pins.sel,
    hlw8012_params.hlw8012_pins.invert_sel_pin_state,
    hlw8012_params.hlw8012_resistors.current,
    hlw8012_params.hlw8012_resistors.voltage_upstream,
    hlw8012_params.hlw8012_resistors.voltage_downstream);

  hlw8012->setVoltageMultiplier(voltage_multiplier);
  hlw8012->setCurrentMultiplier(current_multiplier);
  hlw8012->setActivePowerMultiplier(active_power_multiplier);

  hlw8012->init();

  xTaskCreate(monitorAcTask, "monitorAcTask", 12 * 1024, nullptr, 2, nullptr);
}

void init() {
  ESP_LOGI(kLogTag, "Initializing sensor");
  configureHlw8012(
    {.hlw8012_pins =
       {.cf = GPIO_NUM_14,
        .cf1 = GPIO_NUM_5,
        .sel = GPIO_NUM_4,
        .invert_sel_pin_state = true},
     .hlw8012_resistors =
       {.current = 0.001,
        .voltage_upstream = 5 * 470000,
        .voltage_downstream = 1000},
     .use_interrupts = false,
     .pulse_timeout = 500000});
  initHlw8012();
}

}  // namespace app::modules::sensors