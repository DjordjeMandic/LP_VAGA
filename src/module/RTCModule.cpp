#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <module/RTCModule.hpp>
#include <power/DS3231PowerManager.hpp>
#include <power/twi.hpp>
#include <power/sleep.hpp>
#include <Config.hpp>

/* Static instance of the RTC_DS3231 */
RTC_DS3231 RTCModule::rtc_;

/* Static variable to store the result of the RTC initialization */
bool RTCModule::rtclibBeginResult_ = false;

/* Static variable to track the readiness of the RTC */
bool RTCModule::ready_ = false;

void RTCModule::preBeginPowerOn()
{
    /* Power off the RTC to reset it */
    DS3231PowerManager::powerOff();
    sleep_idle_timeout_millis(1);

    /* Power on the RTC */
    DS3231PowerManager::powerOn();
}

bool RTCModule::begin()
{
    /* Ensure the RTC is powered on before attempting initialization */
    sleep_until(SLEEP_MODE_IDLE, DS3231PowerManager::poweredOn());

    /* Start I2C communication */
    twi_power_on();
    Wire.begin();
    RTCModule::rtclibBeginResult_ = RTCModule::rtc_.begin();

    /* Disable clock output, SQW output, and alarms if initialization succeeded */
    if (RTCModule::rtclibBeginResult_)
    {
        RTCModule::rtc_.disable32K();
        RTCModule::rtc_.disableAlarm(1);
        RTCModule::rtc_.disableAlarm(2);
        RTCModule::rtc_.clearAlarm(1);
        RTCModule::rtc_.clearAlarm(2);
        RTCModule::rtc_.writeSqwPinMode(DS3231_OFF);
    }

    return RTCModule::rtclibBeginResult_;
}

bool RTCModule::ready(unsigned long current_millis)
{
    /* Check if the RTC is powered on and initialized */
    RTCModule::ready_ = DS3231PowerManager::poweredOn(current_millis) && RTCModule::rtclibBeginResult_;
    return RTCModule::ready_;
}

void RTCModule::end()
{
    /* Stop I2C communication and power off the RTC */
    Wire.end();
    twi_power_off();
    DS3231PowerManager::powerOff();
}

bool RTCModule::lostPower()
{
    /* Ensure the RTC is ready before checking power loss */
    if (!RTCModule::ready_)
    {
        return true;
    }

    return RTCModule::rtc_.lostPower();
}

DateTime RTCModule::now() {
    /* Ensure the RTC is ready before reading the current time */
    if (!RTCModule::ready_)
    {
        return DateTime();
    }

    return RTCModule::rtc_.now();
}

bool RTCModule::adjust(const DateTime& dt)
{
    /* Ensure the RTC is ready before adjusting the time */
    if (!RTCModule::ready_)
    {
        return false;
    }

    RTCModule::rtc_.adjust(dt);
    return true;
}

bool RTCModule::setWakeupAlarm(const DateTime& dt, const Ds3231Alarm1Mode alarm_mode)
{
    /* Ensure the RTC is ready before setting the alarm */
    if (!RTCModule::ready_)
    {
        return false;
    }

    return RTCModule::rtc_.setAlarm1(dt, alarm_mode);
}

bool RTCModule::wakeupAlarmFired()
{
    /* Ensure the RTC is ready before checking the alarm status */
    if (!RTCModule::ready_)
    {
        return false;
    }

    return RTCModule::rtc_.alarmFired(1);
}
