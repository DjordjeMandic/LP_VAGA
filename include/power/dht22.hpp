#pragma once

#define DHT22_POWER_DELAY_MS       3000
#define DHT22_POWER_PIN_STATE_OFF  LOW
#define DHT22_POWER_PIN_STATE_ON   HIGH

unsigned long dht22_get_power_on_millis();

void dht22_power_off();

void dht22_power_float();

void dht22_power_on();

bool dht22_power_delay_check();

bool dht22_powered_on();

