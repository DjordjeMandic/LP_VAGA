#pragma once

#include <Arduino.h>
#ifndef ADC_AVCC_SAMPLES_DEFAULT
#define ADC_AVCC_SAMPLES_DEFAULT 6
#endif

void adc_begin();

void adc_end();

uint16_t adc_sample();

void adc_avcc_init();

uint16_t adc_avcc_sample();

uint16_t adc_avcc_sample_average(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

uint16_t adc_avcc_noise_reduced_sample();

uint16_t adc_avcc_noise_reduced_sample_average(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);
