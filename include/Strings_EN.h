#pragma once

#include <Arduino.h>
#include <Version.h>

#define STRING_STARTUP_MESSAGE "UNPAID DEMO " VERSION_SHORT

#define STRING_MULTIPLE_BUTTONS_PRESSED "Multiple buttons pressed"

#define STRING_SCALE_INITIALIZING "Scale initializing"
#define STRING_SCALE_STABILIZING "Scale stabilizing"
#define STRING_SCALE_TARING "Scale taring"

#define STRING_UNKNOWN_COMMAND "Unknown command"

#define STRING_SAVED_TARE_OFFSET_SC_SPACE "Saved tare offset: "

#define STRING_RTC_LOST_POWER "RTC lost power"
#define STRING_SET_CLOCK_OR_USE_TIMER_MODE "Set clock or use timer mode"
#define STRING_RTC_TIME_INVALID "RTC time invalid"
#define STRING_RTC_TIME_SC_SPACE "RTC time: "
#define STRING_READ_RTC_TIME "Read RTC time"
#define STRING_TIMER_ENABLED_SETTING_TIME "Timer enabled. Setting time"

#define STRING_KNOWN_MASS_NOT_FINITE "Known mass not finite"
#define STRING_NEW_SCALE_FACTOR_INVALID "New scale factor invalid"

#define STRING_DHT_INITIALIZING "DHT initializing"

#define STRING_RTC_INITIALIZING "RTC initializing"

#define STRING_GSM_INITIALIZING "GSM initializing"
#define STRING_GSM_REQUIRED "GSM is required to proceed"
#define STRING_GSM_REGISTERING_ON_NETWORK "GSM wait for network"
#define STRING_SENDING_SMS "Sending SMS"

#define STRING_REF_INVALID "Ref invalid"

#define STRING_MEASURE_VOLTAGE_ON_AREF_PIN "Measure voltage on AREF pin"
#define STRING_ENTER_AREF_IN_MV "Enter AREF in mV"
#define STRING_INPUT_SC_SAPCE "Input: "
#define STRING_SAVED_SC_SAPCE "Saved: "
#define STRING_SUPPLY_VOLTAGE_SC_SPACE "Supply voltage: "
#define STRING_SPACE_V " V"

#define STRING_PLACE_SPACE "Place "
#define STRING_SPACE_KG_ON_SCALE_AND_PRESS_TARE " kg on scale and press tare"
#define STRING_SAVED_FACTOR_SC_SPACE "Saved factor: "

#define STRING_TARE_OFFSET_SC_SPACE "Tare offset: "
#define STRING_SCALE_FACTOR_SC_SPACE "Scale factor: "
#define STRING_MEASURED_MASS_SC_SPACE "Measured mass: "
#define STRING_SPACE_KG " kg"

#define STRING_OPERATION_RESULT_SC_SPACE "Operation result: "
#define STRING_OK "ok"
#define STRING_FAIL "fail"
#define STRING_HALTED "halted"