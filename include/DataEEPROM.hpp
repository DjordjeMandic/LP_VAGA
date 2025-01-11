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
     * 
     * @param[out] last_measurement_kg Reference to store the retrieved value.
     */
    static void getLastMeasurementKg(float& last_measurement_kg);

    /**
     * @brief Retrieves the scale calibration value from EEPROM.
     * 
     * @param[out] scale_calibration_value Reference to store the retrieved value.
     */
    static void getScaleCalibrationValue(float& scale_calibration_value);

    /**
     * @brief Retrieves the scale tare offset from EEPROM.
     * 
     * @param[out] scale_tare_offset Reference to store the retrieved value.
     */
    static void getScaleTareOffset(long& scale_tare_offset);

    /**
     * @brief Retrieves the internal ADC reference voltage from EEPROM.
     * 
     * @param[out] internal_reference Reference to store the retrieved value.
     */
    static void getInternalAdcReference(uint16_t& internal_reference);

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

private:
    /* Variables in EEPROM */
    static float EEMEM lastMeasurementKgEemem_;
    static float EEMEM scaleCalibrationValueEemem_;
    static long EEMEM scaleTareOffsetEemem_;
    static uint16_t EEMEM internalAdcReferenceEemem_;
};
