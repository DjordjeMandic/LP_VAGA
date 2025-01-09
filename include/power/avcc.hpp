#pragma once

#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>

#ifndef AVCC_SAMPLES_DEFAULT
#define AVCC_SAMPLES_DEFAULT 6
#endif

#ifndef AVCC_REF_STABILIZATION_DELAY_MS
#define AVCC_REF_STABILIZATION_DELAY_MS 15
#endif

void avcc_adc_init();

uint16_t avcc_sample();

uint16_t avcc_sample_average(uint8_t samples = AVCC_SAMPLES_DEFAULT);

uint16_t avcc_noise_reduced_sample();

uint16_t avcc_noise_reduced_sample_average(uint8_t samples = AVCC_SAMPLES_DEFAULT);
