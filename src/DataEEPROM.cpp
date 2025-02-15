#include <Arduino.h>
#include <avr/eeprom.h>
#include <DataEEPROM.hpp>

/* Static variables in EEPROM */
float EEMEM DataEEPROM::lastMeasurementKgEemem_ = 0.0f;
float EEMEM DataEEPROM::scaleCalibrationValueEemem_ = 1.0f;
long EEMEM DataEEPROM::scaleTareOffsetEemem_ = 0;
uint16_t EEMEM DataEEPROM::internalAdcReferenceEemem_ = 1100U;
uint8_t EEMEM DataEEPROM::smsReportHourOfTheDayEemem_ = 21U;

/* Getters */

float DataEEPROM::getLastMeasurementKg()
{
    eeprom_busy_wait();
    return eeprom_read_float(&lastMeasurementKgEemem_);
}

float DataEEPROM::getScaleCalibrationValue()
{
    eeprom_busy_wait();
    return eeprom_read_float(&scaleCalibrationValueEemem_);
}

long DataEEPROM::getScaleTareOffset()
{
    long scale_tare_offset;
    eeprom_busy_wait();
    eeprom_read_block((void*)&scale_tare_offset, &scaleTareOffsetEemem_, sizeof(scale_tare_offset));
    return scale_tare_offset;
}

uint16_t DataEEPROM::getInternalAdcReference()
{
    eeprom_busy_wait();
    return eeprom_read_word(&internalAdcReferenceEemem_);
}

uint8_t DataEEPROM::getSMSReportHourOfTheDay() {
    eeprom_busy_wait();
    return eeprom_read_byte(&smsReportHourOfTheDayEemem_);
}

/* Setters */

void DataEEPROM::setLastMeasurementKg(const float last_measurement_kg)
{
    eeprom_busy_wait();
    eeprom_update_float(&lastMeasurementKgEemem_, last_measurement_kg);
}

void DataEEPROM::setScaleCalibrationValue(const float scale_calibration_value)
{
    eeprom_busy_wait();
    eeprom_update_float(&scaleCalibrationValueEemem_, scale_calibration_value);
}

void DataEEPROM::setScaleTareOffset(const long scale_tare_offset)
{
    eeprom_busy_wait();
    eeprom_update_block((void*)&scale_tare_offset, &scaleTareOffsetEemem_, sizeof(scale_tare_offset));
}

void DataEEPROM::setInternalAdcReference(const uint16_t internal_reference)
{
    eeprom_busy_wait();
    eeprom_update_word(&internalAdcReferenceEemem_, internal_reference);
}

void DataEEPROM::setSMSReportHourOfTheDay(const uint8_t hour) {
    eeprom_busy_wait();
    eeprom_update_byte(&smsReportHourOfTheDayEemem_, hour);
}
