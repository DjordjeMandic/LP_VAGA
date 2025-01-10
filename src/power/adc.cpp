#include <Arduino.h>
#include <power/adc.hpp>
#include <power/sleep.hpp>

/* Empty ADC conversion complete interrupt to clear ADC interrupt flag (ADIF) */
EMPTY_INTERRUPT (ADC_vect);

/* AVcc as reference and Vbg as input */
#define adc_avcc_mux_set() (ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1))

void adc_begin()
{
    power_adc_enable();

    /* ADC clock: 8 MHz / 64 = 125 KHz */
    ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADEN);

    /* External reference connected to AREF pin and GND channel */
    ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1) | bit(MUX0); 
}

void adc_end()
{
    ADCSRA = 0;
    power_adc_disable();
} 

/**
 * @brief Performs a single ADC conversion on the current channel.
 *        The function starts the conversion, puts the microcontroller into ADC noise reduction sleep mode, 
 *        and returns the result.
 * @return 16-bit result of the ADC conversion.
 * 
 * @note This function assumes that the ADC is already initialized with `adc_begin()`
 */
uint16_t adc_sample()
{
    /* Start the ADC conversion, ADSC will be cleared once done */
    bitSet(ADCSRA, ADSC);

    /* Enable ADC conversion complete interrupt to wake from SLEEP_MODE_ADC */
    bitSet(ADCSRA, ADIE);
    
    /* Sleep until the conversion finishes (ADSC is cleared) */
    sleep_while(SLEEP_MODE_ADC, bit_is_set(ADCSRA, ADSC));

    /* Disable ADC conversion complete interrupt */
    bitClear(ADCSRA, ADIE);

    return ADC;
}

void adc_avcc_init(uint8_t stabilization_delay_ms)
{
    adc_begin();
    
    /* Set adc mux and do one dummy sample */
    adc_avcc_mux_set();
    adc_sample();

    /* Wait for Vref to stabilize */
    sleep_idle_timeout_millis(stabilization_delay_ms);
}

uint16_t adc_avcc_sample()
{
    /* Set adc mux and do one dummy sample */
    adc_avcc_mux_set();
    adc_sample();

    /* Return result of new sample */
    return adc_sample();
}

uint16_t adc_avcc_sample_average(uint8_t samples)
{
    if (samples == 0)
    {
        return 0;
    }

    uint32_t totalAdcReading = 0;

    /*  Set adc mux and do one dummy sample */
    adc_avcc_mux_set();
    adc_sample();

    for (uint8_t i = 0; i < samples; i++)
    {
        totalAdcReading += adc_sample();
    }

    /* Return average */
    return (totalAdcReading + (samples / 2)) / samples;
}
