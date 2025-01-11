#include <Arduino.h>
#include <DHT.h>
#include <module/dht.hpp>
#include <power/dht22.hpp>
#include <Config.hpp>

/* Static instance of the DHT sensor */
DHT DHTModule::dht_(DHT22_DATA_PIN, DHT_TYPE);

void DHTModule::begin(uint8_t pullup_time_us)
{
    /* Power off the sensor to reset it */
    DHT22PowerManager::power_off();
    delay(1);

    /* Power on the sensor and initialize it with the specified pull-up time */
    DHT22PowerManager::power_on();
    DHTModule::dht_.begin(pullup_time_us);
}

void DHTModule::end()
{
    /* Set the data pin to input mode and power off the sensor */
    pinMode(DHT22_DATA_PIN, INPUT);
    DHT22PowerManager::power_off();
}

bool DHTModule::ready()
{
    /* Check if the sensor is powered on and ready */
    return DHT22PowerManager::powered_on() && DHTModule::dht_.read();
}

bool DHTModule::read(bool force)
{
    /* Ensure the sensor is powered on before reading data */
    if (!DHTModule::ensurePoweredOn())
    {
        return false;
    }
    return DHTModule::dht_.read(force);
}

float DHTModule::readTemperature()
{
    /* Ensure the sensor is powered on before reading temperature */
    if (!DHTModule::ensurePoweredOn())
    {
        return NAN;
    }
    return DHTModule::dht_.readTemperature();
}

float DHTModule::readHumidity()
{
    /* Ensure the sensor is powered on before reading humidity */
    if (!DHTModule::ensurePoweredOn())
    {
        return NAN;
    }
    return DHTModule::dht_.readHumidity();
}

bool DHTModule::ensurePoweredOn()
{
    /* Check if the sensor is currently powered on */
    return DHT22PowerManager::powered_on();
}
