#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <module/rtc.hpp>
#include <power/ds3231.hpp>
#include <power/twi.hpp>
#include <config.hpp>

/// @brief RTC_DS3231 object
static RTC_DS3231 rtc_;

/// @brief True if RTC_DS3231::begin() succeeded
static bool rtclib_begin_result_ = false;

void rtc_pre_begin_power_on()
{
    ds3231_power_off();
    delay(1);
    ds3231_power_on();
    twi_power_on();
    Wire.begin();
}

bool rtc_begin()
{
    rtc_return_if_not_powered_on(false);

    rtclib_begin_result_ = rtc_.begin();

    /* disable clock output, sqw output and alarms */
    if (rtclib_begin_result_)
    {
        rtc_.disable32K();
        rtc_.disableAlarm(1);
        rtc_.disableAlarm(2);
        rtc_.clearAlarm(1);
        rtc_.clearAlarm(2);
        rtc_.writeSqwPinMode(DS3231_OFF);
    }

    return rtclib_begin_result_;
}

bool rtc_ready()
{
    return ds3231_powered_on() && rtclib_begin_result_;
}

void rtc_end()
{
    Wire.end();
    twi_power_off();
    ds3231_power_off();
}

bool rtc_lost_power()
{
    rtc_return_if_not_ready(true);

    return rtc_.lostPower();
}

DateTime rtc_now()
{
    rtc_return_if_not_ready(DateTime());

    return rtc_.now();
}

bool rtc_adjust(const DateTime& dt)
{
    rtc_return_if_not_ready(false);

    rtc_.adjust(dt);
    
    return true;
}

bool rtc_set_wakeup_alarm(const DateTime& dt, Ds3231Alarm1Mode alarm_mode)
{
    rtc_return_if_not_ready(false);

    return rtc_.setAlarm1(dt, alarm_mode);
}

bool rtc_wakeup_alarm_fired()
{
    rtc_return_if_not_ready(false);

    return rtc_.alarmFired(1);
}

// /* TODO: implement if needed */
// DateTime rtc_get_wakeup_alarm()
// {
//     rtc_return_if_not_ready(DateTime());

//     return rtc_.getAlarm1();
// }

// /* TODO: implement if needed */
// Ds3231Alarm1Mode rtc_get_wakeup_alarm_mode()
// {
//     /* default if alarm mode cannot be read */
//     rtc_return_if_not_ready(Ds3231Alarm1Mode::DS3231_A1_Date);

//     return rtc_.getAlarm1Mode();
// }
