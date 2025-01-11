#pragma once

#include <Arduino.h>
#include <power/SIM800PowerManager.hpp>

#define gsm_return_if_not_powered_on(x) { if (!SIM800PowerManager::powered_on()) { return x; } }
#define gsm_return_if_not_ready(x) { if (!gsm_ready()) { return x; } }

void gsm_pre_begin_power_on();

bool gsm_begin();

bool gsm_ready();

void gsm_end();