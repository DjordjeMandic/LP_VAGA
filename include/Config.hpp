#pragma once

#include <Arduino.h>

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 38400
#endif

#define RTC_INT_PIN 2

#define ACCEL_INT_PIN 3

#ifndef DS3231_POWER_PIN
#define DS3231_POWER_PIN 4
#endif

#ifndef GSM_POWER_PIN
#define GSM_POWER_PIN 5
#endif

#ifndef GSM_TX_PIN
#define GSM_TX_PIN 6
#endif

#ifndef GSM_RX_PIN
#define GSM_RX_PIN 7
#endif

#ifndef HX711_CLK_PIN
#define HX711_CLK_PIN 8
#endif

#ifndef HX711_DOUT_PIN
#define HX711_DOUT_PIN 9
#endif

#ifndef HX711_POWER_PIN
#define HX711_POWER_PIN 10
#endif

#ifndef HX711_GAIN_FACTOR
#define HX711_GAIN_FACTOR 128
#endif

#ifndef DHT_POWER_PIN
#define DHT_POWER_PIN 11
#endif

#ifndef DHT_DATA_PIN
#define DHT_DATA_PIN 12
#endif

#ifndef BUTTON_CALIBRATE_PIN
#define BUTTON_CALIBRATE_PIN A0
#endif

#ifndef BUTTON_TARE_PIN
#define BUTTON_TARE_PIN A1
#endif

#ifndef BUTTON_TIMER_MODE
#define BUTTON_TIMER_MODE A2
#endif