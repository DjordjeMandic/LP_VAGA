#include <Arduino.h>
#include <avr/eeprom.h>
#include <Data.hpp>

float EEMEM last_measurement_kg_eemem_;
float EEMEM scale_calibration_value_eemem_;
long EEMEM scale_tare_offset_eemem_;

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