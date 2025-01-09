#include <Arduino.h>
#include <power/dht22.hpp>
#include <Config.hpp>

static unsigned long dht22_power_on_millis_ = 0;

unsigned long dht22_get_power_on_millis()
{
    return dht22_power_on_millis_;
}

void dht22_power_off()
{
    pinMode(DHT22_POWER_PIN, OUTPUT);
    digitalWrite(DHT22_POWER_PIN, DHT22_POWER_PIN_STATE_OFF);
}

void dht22_power_float()
{
    pinMode(DHT22_POWER_PIN, INPUT);
}

void dht22_power_on()
{
    pinMode(DHT22_POWER_PIN, OUTPUT);
    digitalWrite(DHT22_POWER_PIN, DHT22_POWER_PIN_STATE_ON); 
    dht22_power_on_millis_ = millis();
}

bool dht22_power_delay_check()
{
    return millis() - dht22_power_on_millis_ >= DHT22_POWER_DELAY_MS;
}

bool dht22_powered_on()
{
    return (digitalRead(DHT22_POWER_PIN) == DHT22_POWER_PIN_STATE_ON) && dht22_power_delay_check();
}
