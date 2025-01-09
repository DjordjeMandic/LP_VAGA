#include <Arduino.h>
#include <HX711.h>
#include <module/scale.hpp>
#include <power/hx711.hpp>
#include <Config.hpp>

static HX711 scale_;

void scale_begin()
{
    hx711_power_off();
    delay(1);
    hx711_power_on();
    scale_.begin(HX711_DOUT_PIN, HX711_CLK_PIN, HX711_GAIN_FACTOR);
}

void scale_end()
{
    pinMode(HX711_DOUT_PIN, INPUT);
    pinMode(HX711_CLK_PIN, INPUT);
    hx711_power_off();
}

bool scale_ready()
{
    return hx711_powered_on() && scale_.is_ready();
}

bool scale_wait_ready(unsigned long delay_ms)
{
    scale_return_if_not_powered_on(false);

    scale_.wait_ready(delay_ms);

    return true;
}

bool scale_wait_ready_retry(int retries, unsigned long delay_ms)
{
    scale_return_if_not_powered_on(false);

    return scale_.wait_ready_retry(retries, delay_ms);
}

bool scale_wait_ready_timeout(unsigned long timeout, unsigned long delay_ms)
{
    scale_return_if_not_powered_on(false);

    return scale_.wait_ready_timeout(timeout, delay_ms);
}

long scale_read()
{
    scale_return_if_not_powered_on(0);
    
    return scale_.read();
}

long scale_read_average(uint8_t times)
{
    scale_return_if_not_powered_on(0);
    
    return scale_.read_average(times);
}

double scale_get_value(uint8_t times)
{
    scale_return_if_not_powered_on(0.0);
    
    return scale_.get_value(times);
}

float scale_get_units(uint8_t times)
{
    scale_return_if_not_powered_on(NAN);

    return scale_.get_units(times);
}

bool scale_tare(uint8_t times)
{
    scale_return_if_not_powered_on(false);

    scale_.tare(times);

    return true;
}

void scale_set_scale(float scale)
{
    scale_.set_scale(scale);
}

float scale_get_scale()
{
    return scale_.get_scale();
}

void scale_set_offset(long offset)
{
    scale_.set_offset(offset);
}

long scale_get_offset()
{
    return scale_.get_offset();
}
