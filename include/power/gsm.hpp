#pragma once

#define GSM_POWER_DELAY_MS      500
#define GSM_POWER_PIN_STATE_OFF LOW
#define GSM_POWER_PIN_STATE_ON  HIGH

unsigned long gsm_get_power_on_millis();

void gsm_power_off();

void gsm_power_on();

bool gsm_power_delay_check();

bool gsm_powered_on();