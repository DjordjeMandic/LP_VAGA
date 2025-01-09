#pragma once

#include <Arduino.h>
#include <HX711.h>
#include <power/hx711.hpp>

#warning Stabilizing time of 3-5 seconds should be included (keep reading at 10hz for 3-5 seconds constantly)

#define SCALE_HX711_NOT_POWERED_ON_RETURN(x) { if (!hx711_powered_on()) { return x; } }

#define SCALE_HX711_DOUT_PIN      7
#define SCALE_HX711_CLK_PIN       6
#define SCALE_HX711_GAIN_FACTOR   128

void scale_begin();

void scale_end();

bool scale_ready();

bool scale_wait_ready(unsigned long delay_ms = 0);

bool scale_wait_ready_retry(int retries = 3, unsigned long delay_ms = 0);

bool scale_wait_ready_timeout(unsigned long timeout = 1000, unsigned long delay_ms = 0);

long scale_read();

long scale_read_average(uint8_t times = 10);

double scale_get_value(uint8_t times = 1);

float scale_get_units(uint8_t times = 1);

bool scale_tare(uint8_t times = 10);

void scale_set_scale(float scale = 1.f);

float scale_get_scale();

void scale_set_offset(long offset = 0);

long scale_get_offset();