#pragma once

#include <Arduino.h>

#define HX711_POWER_DELAY_MS        500
#define HX711_POWER_PIN_STATE_OFF   LOW
#define HX711_POWER_PIN_STATE_ON    HIGH

unsigned long hx711_get_power_on_millis();

void hx711_power_off();

void hx711_power_float();

void hx711_power_on();

bool hx711_power_delay_check();

bool hx711_powered_on();