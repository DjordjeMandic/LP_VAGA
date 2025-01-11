#include <Arduino.h>
#include <LowPower.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <power/sleep.hpp>
#include <serial.hpp>

void sleep_idle_timeout_millis(const unsigned long sleep_time_ms)
{
    unsigned long start = millis();

    /* Sleep until the specified sleep time is reached */
    sleep_while(SLEEP_MODE_IDLE, (millis() - start) < sleep_time_ms);
}

void sleep_power_down_80ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_95ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_125ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_185ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_315ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_565ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
}

void sleep_power_down_1065ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
}

void sleep_power_down_2065ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_ON);
}

void sleep_power_down_4065ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_ON);
}

void sleep_power_down_8065ms_adc_off_bod_on()
{
    serial_flush();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}
