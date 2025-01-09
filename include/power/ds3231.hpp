#pragma once

#include <Arduino.h>

#define DS3231_POWER_DELAY_MS       500
#define DS3231_POWER_PIN_STATE_OFF  LOW
#define DS3231_POWER_PIN_STATE_ON   HIGH

unsigned long ds3231_get_power_on_millis();

void ds3231_power_off();

void ds3231_power_float();

void ds3231_power_on();

bool ds3231_power_delay_check();

bool ds3231_powered_on();