#include <Arduino.h>
#include <power/ADCHelper.hpp>
#include <power/sleep.hpp>

/* Empty ADC conversion complete interrupt to clear ADC interrupt flag (ADIF) */
EMPTY_INTERRUPT (ADC_vect);

void ADCHelper::begin()
{
    power_adc_enable();

    /* ADC clock: 8 MHz / 64 = 125 KHz */
    ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADEN);

    /* External reference connected to AREF pin and GND channel */
    ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1) | bit(MUX0); 
}

void ADCHelper::end()
{
    ADCSRA = 0;
    power_adc_disable();
}

void ADCHelper::avccInit(const uint8_t stabilization_delay_ms)
{
    ADCHelper::begin();

    /* Set ADC mux and do one dummy sample */
    ADCHelper::setAvccMux();
    ADCHelper::sample();

    /* Wait for Vref to stabilize */
    sleep_idle_timeout_millis(stabilization_delay_ms);
}

uint16_t ADCHelper::avccSample()
{
    /* Set ADC mux and do one dummy sample */
    ADCHelper::setAvccMux();
    ADCHelper::sample();

    /* Return result of new sample */
    return ADCHelper::sample();
}

uint16_t ADCHelper::avccSampleAverage(const uint8_t samples)
{
    if (samples == 0)
    {
        return 0;
    }

    uint32_t totalAdcReading = 0;

    /* Set ADC mux and do one dummy sample */
    ADCHelper::setAvccMux();
    ADCHelper::sample();

    for (uint8_t i = 0; i < samples; i++)
    {
        totalAdcReading += ADCHelper::sample();
    }

    /* Return average */
    return (totalAdcReading + (samples / 2)) / samples;
}

uint16_t ADCHelper::sample()
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

void ADCHelper::setAvccMux()
{
    /* AVcc as reference and Vbg as input */
    ADMUX = bit(REFS0) | bit(MUX3) | bit(MUX2) | bit(MUX1);
}
