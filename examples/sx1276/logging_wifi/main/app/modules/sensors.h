#pragma once

#include <stdint.h>
#include <stdio.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace app::modules::sensors {

double getVoltage();
double getCurrent();
double getActivePower();
double getApparentPower();
int getPowerFactor();

void setVoltageMultiplier(double value);
void setCurrentMultiplier(double value);
void setActivePowerMultiplier(double value);

void updateCurrent();
cJSON *telemetryCallback(cJSON *payload, uint32_t seq);

void init();
}  // namespace app::modules::sensors