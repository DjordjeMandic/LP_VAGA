#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <module/rtc.hpp>
#include <power/ds3231.hpp>
#include <power/twi.hpp>
#include <power/sleep.hpp>
#include <config.hpp>

/* Static instance of the RTC_DS3231 */
RTC_DS3231 RTCModule::rtc_;

/* Static variable to store the result of the RTC initialization */
bool RTCModule::rtclibBeginResult_ = false;

void RTCModule::preBeginPowerOn()
{
    /* Power off the RTC to reset it */
    DS3231PowerManager::power_off();
    delay(1);

    /* Power on the RTC and start I2C communication */
    DS3231PowerManager::power_on();
    twi_power_on();
    Wire.begin();
}

bool RTCModule::begin()
{
    /* Ensure the RTC is powered on before attempting initialization */
    sleep_until(SLEEP_MODE_IDLE, DS3231PowerManager::powered_on());

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

bool RTCModule::ready()
{
    /* Check if the RTC is powered on and initialized */
    return DS3231PowerManager::powered_on() && RTCModule::rtclibBeginResult_;
}

void RTCModule::end()
{
    /* Stop I2C communication and power off the RTC */
    Wire.end();
    twi_power_off();
    DS3231PowerManager::power_off();
}

bool RTCModule::lostPower()
{
    /* Ensure the RTC is ready before checking power loss */
    if (!RTCModule::ready())
    {
        return true;
    }

    return RTCModule::rtc_.lostPower();
}

DateTime RTCModule::now() {
    /* Ensure the RTC is ready before reading the current time */
    if (!RTCModule::ready())
    {
        return DateTime();
    }

    return RTCModule::rtc_.now();
}

bool RTCModule::adjust(const DateTime& dt)
{
    /* Ensure the RTC is ready before adjusting the time */
    if (!RTCModule::ready())
    {
        return false;
    }

    RTCModule::rtc_.adjust(dt);
    return true;
}

bool RTCModule::setWakeupAlarm(const DateTime& dt, const Ds3231Alarm1Mode alarm_mode)
{
    /* Ensure the RTC is ready before setting the alarm */
    if (!RTCModule::ready())
    {
        return false;
    }

    return RTCModule::rtc_.setAlarm1(dt, alarm_mode);
}

bool RTCModule::wakeupAlarmFired()
{
    /* Ensure the RTC is ready before checking the alarm status */
    if (!RTCModule::ready())
    {
        return false;
    }

    return RTCModule::rtc_.alarmFired(1);
}
