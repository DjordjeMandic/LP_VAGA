#include <Arduino.h>
#include <DHT.h>
#include <module/DHTModule.hpp>
#include <power/DHT22PowerManager.hpp>
#include <power/sleep.hpp>
#include <Config.hpp>

/* Static instance of the DHT sensor */
DHT DHTModule::dht_(DHT22_DATA_PIN, DHT_TYPE);

/* Static variable to track the readiness of the DHT sensor */
bool DHTModule::ready_ = false;

void DHTModule::begin(uint8_t pullup_time_us)
{
    /* Power off the sensor to reset it */
    DHT22PowerManager::powerOff();
    sleep_idle_timeout_millis(1);

    /* Power on the sensor and initialize it with the specified pull-up time */
    DHT22PowerManager::powerOn();
    DHTModule::dht_.begin(pullup_time_us);
}

void DHTModule::end()
{
    DHTModule::ready_ = false;
    /* Set the data pin to input mode and power off the sensor */
    pinMode(DHT22_DATA_PIN, INPUT);
    DHT22PowerManager::powerOff();
}

bool DHTModule::ready(unsigned long current_millis)
{
    /* Check if the sensor is powered on and ready */
    DHTModule::ready_ = DHT22PowerManager::poweredOn(current_millis) && DHTModule::dht_.read();
    return DHTModule::ready_;
}

bool DHTModule::read(bool force)
{
    /* Ensure the sensor is powered on before reading data */
    if (!DHTModule::ready_)
    {
        return false;
    }
    return DHTModule::dht_.read(force);
}

float DHTModule::readTemperature()
{
    /* Ensure the sensor is powered on before reading temperature */
    if (!DHTModule::ready_)
    {
        return NAN;
    }
    return DHTModule::dht_.readTemperature();
}

float DHTModule::readHumidity()
{
    /* Ensure the sensor is powered on before reading humidity */
    if (!DHTModule::ready_)
    {
        return NAN;
    }
    return DHTModule::dht_.readHumidity();
}
