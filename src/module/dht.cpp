#include <Arduino.h>
#include <DHT.h>
#include <module/dht.hpp>
#include <power/dht22.hpp>
#include <config.hpp>

static DHT dht(DHT22_DATA_PIN, DHT_TYPE);

void dht_begin(uint8_t pullup_time_us)
{
    DHT22PowerManager::power_off();
    delay(1);
    DHT22PowerManager::power_on();
    dht.begin(pullup_time_us);
}

void dht_end()
{
    pinMode(DHT22_DATA_PIN, INPUT);
    DHT22PowerManager::power_off();
}

bool dht_ready()
{
    return DHT22PowerManager::powered_on() && dht.read();
}

bool dht_read(bool force)
{
    dht_return_if_not_powered_on(false);
    return dht.read(force);
}

float dht_read_temperature()
{
    dht_return_if_not_powered_on(NAN);
    return dht.readTemperature();
}

float dht_read_humidity()
{
    dht_return_if_not_powered_on(NAN);
    return dht.readHumidity();
}