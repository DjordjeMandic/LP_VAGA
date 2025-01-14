#include <Arduino.h>
#include <HX711.h>
#include <module/ScaleModule.hpp>
#include <power/HX711PowerManager.hpp>
#include <power/sleep.hpp>
#include <Config.hpp>

/* Static instance of the HX711 */
HX711 ScaleModule::scale_;

void ScaleModule::begin()
{
    HX711PowerManager::powerOff();
    sleep_idle_timeout_millis(1);
    
    HX711PowerManager::powerOn();
    ScaleModule::scale_.begin(HX711_DOUT_PIN, HX711_CLK_PIN, HX711_GAIN_FACTOR);
}

void ScaleModule::end()
{
    pinMode(HX711_DOUT_PIN, INPUT);
    pinMode(HX711_CLK_PIN, INPUT);
    HX711PowerManager::powerOff();
}

bool ScaleModule::ready()
{
    return HX711PowerManager::poweredOn() && ScaleModule::scale_.is_ready();
}

bool ScaleModule::waitReady()
{
    if (!HX711PowerManager::poweredOn())
    {
        return false;
    }
    
    /* Timer0 ISR runs every 1024us */
    sleep_until(SLEEP_MODE_IDLE, ScaleModule::ready());

    return true;
}

bool ScaleModule::waitReadyRetry(const int retries, const unsigned long delay_ms)
{
    int count = 0;

    while (count < retries)
    {
        if (ScaleModule::ready())
        {
            return true;
        }

        sleep_idle_timeout_millis(delay_ms);
        count++;
    }

    return false; // Retries exhausted
}

bool ScaleModule::waitReadyTimeout(const unsigned long timeout, const unsigned long delay_ms)
{
    unsigned long millisStarted = millis();
    while (millis() - millisStarted < timeout)
    {
        if (ScaleModule::ready())
        {
            return true;
        }

        /* Timer0 ISR runs every 1024us */
        sleep_while(SLEEP_MODE_IDLE, false);
    }
    return false;
}

bool ScaleModule::stabilize(const uint16_t stabilization_time_ms)
{
    if (!HX711PowerManager::poweredOn())
    {
        return false;
    }

    if (stabilization_time_ms > 0)
    {
        unsigned long elapsedTime = 0;

        while (elapsedTime < stabilization_time_ms)
        {
            /* If sample is not ready, scale will wait for it in blocking active loop */
            ScaleModule::scale_.read(); // Perform the scale reading

            /* Sample rate is 10Hz (100ms) */
            sleep_power_down_95ms_adc_off_bod_on();
            elapsedTime += 95;
        }
    }

    return true;
}

long ScaleModule::read()
{
    if (!HX711PowerManager::poweredOn())
    {
        return 0;
    }

    return ScaleModule::scale_.read();
}

long ScaleModule::readAverage(const uint8_t times)
{
    if (!HX711PowerManager::poweredOn())
    {
        return 0;
    }

    return ScaleModule::scale_.read_average(times);
}

double ScaleModule::getValue(const uint8_t times)
{
    if (!HX711PowerManager::poweredOn())
    {
        return 0.0;
    }

    return ScaleModule::scale_.get_value(times);
}

float ScaleModule::getUnits(const uint8_t times)
{
    if (!HX711PowerManager::poweredOn())
    {
        return NAN;
    }

    return ScaleModule::scale_.get_units(times);
}

bool ScaleModule::tare(const uint8_t times)
{
    if (!HX711PowerManager::poweredOn())
    {
        return false;
    }

    ScaleModule::scale_.tare(times);

    return true;
}

void ScaleModule::setScale(const float scale)
{
    ScaleModule::scale_.set_scale(scale);
}

float ScaleModule::getScale()
{
    return ScaleModule::scale_.get_scale();
}

void ScaleModule::setOffset(const long offset)
{
    ScaleModule::scale_.set_offset(offset);
}

long ScaleModule::getOffset()
{
    return ScaleModule::scale_.get_offset();
}
