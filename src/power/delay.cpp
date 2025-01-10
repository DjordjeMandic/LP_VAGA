#include <Arduino.h>
#include <LowPower.h>
#include <power/delay.hpp>

void delay_80ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON);
}

void delay_95ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_ON);
}

void delay_125ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
}

void delay_185ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
}

void delay_315ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
}

void delay_565ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
}

void delay_1065ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
}

void delay_2065ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_ON);
}

void delay_4065ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_ON);
}

void delay_8065ms_power_down_adc_off_bod_on()
{
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}
