#pragma once

#include <Arduino.h>
#include <avr/eeprom.h>

/**
 * @brief A class to manage reading and writing calibration and measurement data to EEPROM.
 * 
 * This class provides static methods to get and set values such as the last measurement, 
 * scale calibration value, tare offset, and internal ADC reference. It ensures proper 
 * handling of EEPROM read and write operations.
 */
class DataEEPROM
{
public:
    /**
     * @brief Retrieves the last measurement in kilograms from EEPROM.
     */
    static float getLastMeasurementKg();

    /**
     * @brief Retrieves the scale calibration value from EEPROM.
     */
    static float getScaleCalibrationValue();

    /**
     * @brief Retrieves the scale tare offset from EEPROM.
     */
    static long getScaleTareOffset();

    /**
     * @brief Retrieves the internal ADC reference voltage from EEPROM.
     */
    static uint16_t getInternalAdcReference();

    /**
     * @brief Retrieves the SMS report hour of the day from EEPROM.
     */
    static uint8_t getSMSReportHourOfTheDay();

    /**
     * @brief Updates the last measurement in kilograms in EEPROM.
     * 
     * @param[in] last_measurement_kg The value to store in EEPROM.
     */
    static void setLastMeasurementKg(const float last_measurement_kg);

    /**
     * @brief Updates the scale calibration value in EEPROM.
     * 
     * @param[in] scale_calibration_value The value to store in EEPROM.
     */
    static void setScaleCalibrationValue(const float scale_calibration_value);

    /**
     * @brief Updates the scale tare offset in EEPROM.
     * 
     * @param[in] scale_tare_offset The value to store in EEPROM.
     */
    static void setScaleTareOffset(const long scale_tare_offset);

    /**
     * @brief Updates the internal ADC reference voltage in EEPROM.
     * 
     * @param[in] internal_reference The value to store in EEPROM.
     */
    static void setInternalAdcReference(const uint16_t internal_reference);

    /**
     * @brief Updates the SMS report hour of the day in EEPROM.
     * 
     * @param[in] hour The value to store in EEPROM.
     */
    static void setSMSReportHourOfTheDay(const uint8_t hour);

private:
    /* Variables in EEPROM */
    static float EEMEM lastMeasurementKgEemem_;
    static float EEMEM scaleCalibrationValueEemem_;
    static long EEMEM scaleTareOffsetEemem_;
    static uint16_t EEMEM internalAdcReferenceEemem_;
    static uint8_t EEMEM smsReportHourOfTheDayEemem_;
};
