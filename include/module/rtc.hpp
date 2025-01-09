#pragma once

#include <Arduino.h>
#include <power/ds3231.hpp>

#define rtc_return_if_not_powered_on(x) { if (!ds3231_powered_on()) { return x; } }
#define rtc_return_if_not_ready(x) { if (!rtc_ready()) { return x; } }

void rtc_pre_begin_power_on();

bool rtc_begin();

bool rtc_ready();

void rtc_end();

bool rtc_lost_power();

DateTime rtc_now();

bool rtc_adjust(const DateTime& dt);

bool rtc_set_wakeup_alarm(const DateTime& dt, Ds3231Alarm1Mode alarm_mode);

bool rtc_wakeup_alarm_fired();

DateTime rtc_get_wakeup_alarm();

Ds3231Alarm1Mode rtc_get_wakeup_alarm_mode();