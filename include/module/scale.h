#pragma once

#include <Arduino.h>
#include <HX711.h>
#include <power/hx711.h>

#define SCALE_HX711_NOT_POWERED_ON_RETURN(x) { if (!hx711_powered_on()) { return x; } }

#define SCALE_HX711_DOUT_PIN      7
#define SCALE_HX711_CLK_PIN       6
#define SCALE_HX711_GAIN_FACTOR   128

HX711 scale_;

void scale_begin()
{
    hx711_power_off();
    delay(1);
    hx711_power_on();
    scale_.begin(SCALE_HX711_DOUT_PIN, SCALE_HX711_CLK_PIN, SCALE_HX711_GAIN_FACTOR);
}

void scale_end()
{
    pinMode(SCALE_HX711_DOUT_PIN, INPUT);
    pinMode(SCALE_HX711_CLK_PIN, INPUT);
    hx711_power_off();
}

bool scale_ready()
{
    return hx711_powered_on() && scale_.is_ready();
}

bool scale_wait_ready(unsigned long delay_ms = 0)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(false);

    scale_.wait_ready(delay_ms);

    return true;
}

bool scale_wait_ready_retry(int retries = 3, unsigned long delay_ms = 0)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(false);

    return scale_.wait_ready_retry(retries, delay_ms);
}

bool scale_wait_ready_timeout(unsigned long timeout = 1000, unsigned long delay_ms = 0)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(false);

    return scale_.wait_ready_timeout(timeout, delay_ms);
}

long scale_read()
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(0);
    
    return scale_.read();
}

long scale_read_average(uint8_t times = 10)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(0);
    
    return scale_.read_average(times);
}

double scale_get_value(uint8_t times = 1)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(0.0);
    
    return scale_.get_value(times);
}

float scale_get_units(uint8_t times = 1)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(NAN);

    return scale_.get_units(times);
}

bool scale_tare(uint8_t times = 10)
{
    SCALE_HX711_NOT_POWERED_ON_RETURN(false);

    scale_.tare(times);

    return true;
}

void scale_set_scale(float scale = 1.f)
{
    scale_.set_scale(scale);
}

float scale_get_scale()
{
    return scale_.get_scale();
}

void scale_set_offset(long offset = 0)
{
    scale_.set_offset(offset);
}

long scale_get_offset()
{
    return scale_.get_offset();
}
