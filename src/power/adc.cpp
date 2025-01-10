#include <Arduino.h>
#include <LowPower.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <power/adc.hpp>

EMPTY_INTERRUPT (ADC_vect);

#define adc_avcc_mux_set() (ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1))

void adc_begin()
{
    power_adc_enable();
    ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADEN); // 8 MHz / 64 = 125 KHz
    ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1) | bit(MUX0); // External reference connected to AREF pin and GND channel
}

void adc_end()
{
    ADCSRA = 0;
    power_adc_disable();
}

uint16_t adc_sample()
{
    bitSet(ADCSRA, ADSC); // Start the ADC conversion
    loop_until_bit_is_clear(ADCSRA, ADSC); // Wait for the ADC conversion to complete
    return ADC;
}

void adc_avcc_init()
{
    adc_begin();
    adc_avcc_mux_set();
    adc_sample();
    LowPower.idle(SLEEP_15MS, /* sleep for 15 ms, no startup delay since clock won't be off */
                ADC_ON, /* keep adc on */
                TIMER2_ON, /* do not modify timer2 state */
                TIMER1_ON, /* do not modify timer1 state */
                TIMER0_OFF, /* turn off timer0 to prevent wakeup (used for micros() and millis()) */
                SPI_ON, /* do not modify SPI state */
                USART0_ON, /* do not modify USART0 state */
                TWI_ON); /* do not modify TWI state */
}

uint16_t adc_avcc_sample()
{
    adc_avcc_mux_set();
    adc_sample();
    return adc_sample();
}

uint16_t adc_avcc_sample_average(uint8_t samples)
{
    if (samples == 0)
    {
        samples++;
    }
    uint32_t totalAdcReading = 0;

    // Set adc mux and do one dummy sample
    adc_avcc_mux_set();
    adc_sample();

    for (uint8_t i = 0; i < samples; i++)
    {
        totalAdcReading += adc_sample();
    }

    return (totalAdcReading + (samples / 2)) / samples;
}

uint16_t adc_avcc_noise_reduced_sample()
{
    // set mux and do one dummy sample
    adc_avcc_mux_set();
    adc_sample();

    bitSet(ADCSRA, ADIF); // turn off any pending interrupt 
    bitSet(ADCSRA, ADSC); // Start the ADC conversion

    do
    {
        LowPower.adcNoiseReduction(SLEEP_FOREVER, ADC_ON, TIMER2_OFF);
    } while (bit_is_set(ADCSRA, ADSC)); // Sleep until ADC conversion is complete

    return ADC; // Return ADC result
}

uint16_t adc_avcc_noise_reduced_sample_average(uint8_t samples)
{
    if (samples == 0)
    {
        samples++;
    }

    uint32_t totalAdcReading = 0;

    // Set adc mux and do one dummy sample
    adc_avcc_mux_set();
    adc_sample();

    bitSet(ADCSRA, ADIF); // turn off any pending interrupt 

    for (uint8_t i = 0; i < samples; i++)
    {
        bitSet(ADCSRA, ADSC); // Start the ADC conversion

        do
        {
            LowPower.adcNoiseReduction(SLEEP_FOREVER, ADC_ON, TIMER2_OFF);
        } while (bit_is_set(ADCSRA, ADSC)); // Sleep until ADC conversion is complete

        totalAdcReading += ADC;
    }

    return (totalAdcReading + (samples / 2)) / samples; // Return ADC result
}
