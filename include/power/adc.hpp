#pragma once

#include <Arduino.h>
#ifndef ADC_AVCC_SAMPLES_DEFAULT
#define ADC_AVCC_SAMPLES_DEFAULT 6
#endif

#ifndef ADC_AVCC_REF_STABILIZATION_DELAY_MS
#define ADC_AVCC_REF_STABILIZATION_DELAY_MS 15
#endif

void adc_begin();

void adc_end();

uint16_t adc_sample();

void adc_avcc_init(uint8_t stabilization_delay_ms = ADC_AVCC_REF_STABILIZATION_DELAY_MS);

uint16_t adc_avcc_sample();

uint16_t adc_avcc_sample_average(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

uint16_t adc_avcc_noise_reduced_sample();

uint16_t adc_avcc_noise_reduced_sample_average(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);
