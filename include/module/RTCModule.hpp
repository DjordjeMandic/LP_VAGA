#pragma once

#include <Arduino.h>
#include <RTClib.h>
#include <power/DS3231PowerManager.hpp>

/**
 * @brief A class to manage operations for a single DS3231 RTC module.
 * 
 * This class provides static methods to initialize, read the current time,
 * set alarms, and manage the power state of the DS3231 RTC module. It ensures
 * proper handling of power states and readiness checks before performing operations.
 */
class RTCModule
{
public:
    /**
     * @brief Prepares the RTC module for initialization by shortly powering it off before powering it on.
     */
    static void preBeginPowerOn();
    
    /**
     * @brief Initializes the RTC module.
     * 
     * This method waits for RTC to powers on, starts I2C communication, and disables 
     * unnecessary features like clock output and alarms.
     * 
     * @note This method is blocking until power-on is complete. Use `preBeginPowerOn()`
     *       to prepare the RTC for initialization. To prevent blocking, before calling this method,
     *       ensure the RTC is powered on by calling `DS3231PowerManager::poweredOn()`.
     * 
     * @return `true` if initialization was successful, otherwise `false`.
     */
    static bool begin();
    
    /**
     * @brief Checks if the RTC module is ready for use.
     * 
     * This method verifies that the RTC is powered on and that initialization 
     * was successful.
     * 
     * @param[in] current_millis The current time in milliseconds. If not provided, 
     * `millis()` will be used.
     * 
     * @return `true` if the RTC is ready, otherwise `false`.
     */
    static bool ready(unsigned long current_millis = millis());
    
    /**
     * @brief Powers off the RTC module and stops I2C communication.
     */
    static void end();
    
    /**
     * @brief Checks if the RTC module has lost power.
     * 
     * This method verifies if the RTC lost power and reset its time.
     * 
     * @return `true` if the RTC lost power, otherwise `false`.
     */
    static bool lostPower();
    
    /**
     * @brief Returns the current date and time from the RTC.
     * 
     * @return A `DateTime` object representing the current date and time.
     */
    static DateTime now();
    
    /**
     * @brief Adjusts the RTC time to the specified date and time.
     * 
     * @param[in] dt The desired date and time to set.
     * @return `true` if the adjustment was successful, otherwise `false`.
     */
    static bool adjust(const DateTime& dt);
    
    /**
     * @brief Sets a wakeup alarm on the RTC.
     * 
     * @param[in] dt The desired date and time for the alarm.
     * @param[in] alarm_mode The mode for the alarm (see `Ds3231Alarm1Mode`).
     * @return `true` if the alarm was set successfully, otherwise `false`.
     */
    static bool setWakeupAlarm(const DateTime& dt, const Ds3231Alarm1Mode alarm_mode);
    
    /**
     * @brief Checks if the wakeup alarm has fired.
     * 
     * @return `true` if the alarm has fired, otherwise `false`.
     */
    static bool wakeupAlarmFired();

private:
    static RTC_DS3231 rtc_; /**< Static instance of the RTC_DS3231 class for managing RTC operations. */
    static bool rtclibBeginResult_; /**< Stores the result of the RTC initialization. */
};
