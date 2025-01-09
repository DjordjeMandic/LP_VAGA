#include <Arduino.h>
#include <avr/eeprom.h>
#include <Data.hpp>

/* Variables in EEPROM */

float EEMEM last_measurement_kg_eemem_;
float EEMEM scale_calibration_value_eemem_;
long EEMEM scale_tare_offset_eemem_;
uint16_t EEMEM internal_adc_reference_eemem_;

/* End of variables in EEPROM */

/* Getters */

void data_get_last_measurement_kg(float& last_measurement_kg)
{
    eeprom_busy_wait();
    last_measurement_kg = eeprom_read_float(&last_measurement_kg_eemem_);
}

void data_get_scale_calibration_value(float& scale_calibration_value)
{
    eeprom_busy_wait();
    scale_calibration_value = eeprom_read_float(&scale_calibration_value_eemem_);
}

void data_get_scale_tare_offset(long& scale_tare_offset)
{
    eeprom_busy_wait();
    eeprom_read_block((void *)&scale_tare_offset, &scale_tare_offset_eemem_, sizeof(scale_tare_offset));
}

void data_get_internal_adc_reference(uint16_t& internal_reference)
{
    eeprom_busy_wait();
    internal_reference = eeprom_read_word(&internal_adc_reference_eemem_);
}

/* End of getters */

/* Setters */

void data_set_last_measurement_kg(const float& last_measurement_kg)
{
    eeprom_busy_wait();
    eeprom_update_float(&last_measurement_kg_eemem_, last_measurement_kg);
}

void data_set_scale_calibration_value(const float& scale_calibration_value)
{
    eeprom_busy_wait();
    eeprom_update_float(&scale_calibration_value_eemem_, scale_calibration_value);
}

void data_set_scale_tare_offset(const long& scale_tare_offset)
{
    eeprom_busy_wait();
    eeprom_update_block((void *)&scale_tare_offset, &scale_tare_offset_eemem_, sizeof(scale_tare_offset));
}

void data_set_internal_adc_reference(const uint16_t& internal_reference)
{
    eeprom_busy_wait();
    eeprom_update_word(&internal_adc_reference_eemem_, internal_reference);
}

/* End of setters */