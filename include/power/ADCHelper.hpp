#pragma once

#include <Arduino.h>
#include <Config.hpp>

/**
 * @brief A class to manage operations for the ADC module.
 * 
 * This class provides static methods to initialize, perform single and 
 * averaged ADC conversions, and manage the power state of the ADC.
 */
class ADCHelper
{
public:
    /**
     * @brief Initializes the ADC by enabling power and setting the prescaler.
     *        Uses AVcc as the reference voltage and configures the initial input channel (GND).
     */
    static void begin();

    /**
     * @brief Disables the ADC by clearing control registers and disabling power.
     */
    static void end();

    /**
     * @brief Initializes the ADC for measuring the AVcc reference voltage.
     *        The function sets the correct ADC MUX for AVcc measurement, performs a dummy sample,
     *        and waits for the reference to stabilize.
     * 
     * @param[in] stabilization_delay_ms Delay in milliseconds to allow Vref stabilization.
     *        Defaults to @p `ADC_AVCC_REF_STABILIZATION_DELAY_MS` if not provided.
     */
    static void avccInit(const uint8_t stabilization_delay_ms = ADC_AVCC_REF_STABILIZATION_DELAY_MS);

    /**
     * @brief Performs a single ADC conversion with AVcc as the reference.
     *        The function sets the correct MUX for AVcc measurement, performs a dummy sample,
     *        and then returns the result of a new sample.
     * 
     * @return 16-bit result of the ADC conversion.
     * 
     * @note This function assumes that the ADC is already initialized with `avccInit()`
     */
    static uint16_t avccSample();

    /**
     * @brief Computes the average of multiple ADC samples with AVcc as the reference.
     *        A dummy sample is performed first to ensure stability, and then the specified number 
     *        of samples is averaged.
     * 
     * @param[in] samples Number of ADC samples to average. Defaults to @p `ADC_AVCC_SAMPLES_DEFAULT` if not provided.
     * @return 16-bit average result of the ADC samples.
     * 
     * @note This function assumes that the ADC is already initialized with `avccInit()`
     */
    static uint16_t avccSampleAverage(const uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

private:
    /**
     * @brief Performs a single ADC conversion on the current channel.
     *        The function starts the conversion, puts the microcontroller into ADC noise reduction sleep mode, 
     *        and returns the result.
     * 
     * @return 16-bit result of the ADC conversion.
     * 
     * @note This function assumes that the ADC is already initialized with `begin()`
     */
    static uint16_t sample();

    /**
     * @brief Sets the ADC MUX for AVcc measurement.
     */
    static void setAvccMux();
};
