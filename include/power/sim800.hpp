#pragma once

#define SIM800_POWER_DELAY_MS      500
#define SIM800_POWER_PIN_STATE_OFF LOW
#define SIM800_POWER_PIN_STATE_ON  HIGH

unsigned long sim800_get_power_on_millis();

void sim800_power_off();

void sim800_power_on();

bool sim800_power_delay_check();

bool sim800_powered_on();