#pragma once

#include <Arduino.h>
#include <power/dht22.hpp>

#define dht_return_if_not_powered_on(x) { if (!DHT22PowerManager::powered_on()) { return x; } }

void dht_begin(uint8_t pullup_time_us = 55);

void dht_end();

bool dht_ready();

bool dht_read(bool force = false);

float dht_read_temperature();

float dht_read_humidity();