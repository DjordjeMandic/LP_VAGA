#pragma once

#include <Arduino.h>
#include <config.hpp>

/**
 * @brief Initializes the ADC by enabling power and setting the prescaler.
 *        Uses AVcc as the reference voltage and configures the initial input channel (GND).
 */
void adc_begin();

/**
 * @brief Disables the ADC by clearing control registers and disabling power.
 */
void adc_end();

/**
 * @brief Initializes the ADC for measuring the AVcc reference voltage.
 *        The function sets the correct ADC MUX for AVcc measurement, performs a dummy sample,
 *        and waits for the reference to stabilize.
 * @param[in] stabilization_delay_ms Delay in milliseconds to allow Vref stabilization.
 *        Defaults to @p `ADC_AVCC_REF_STABILIZATION_DELAY_MS` if not provided.
 */
void adc_avcc_init(uint8_t stabilization_delay_ms = ADC_AVCC_REF_STABILIZATION_DELAY_MS);

/**
 * @brief Performs a single ADC conversion with AVcc as the reference.
 *        The function sets the correct MUX for AVcc measurement, performs a dummy sample,
 *        and then returns the result of a new sample.
 * @return 16-bit result of the ADC conversion.
 * 
 * @note This function assumes that the ADC is already initialized with `adc_avcc_init()`
 */
uint16_t adc_avcc_sample();

/**
 * @brief Computes the average of multiple ADC samples with AVcc as the reference.
 *        A dummy sample is performed first to ensure stability, and then the specified number 
 *        of samples is averaged.
 * @param[in] samples Number of ADC samples to average. Defaults to @p `ADC_AVCC_SAMPLES_DEFAULT` if not provided.
 * @return 16-bit average result of the ADC samples.
 * 
 * @note This function assumes that the ADC is already initialized with `adc_avcc_init()`
 */
uint16_t adc_avcc_sample_average(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);