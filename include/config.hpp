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

#ifndef SIM800_POWER_PIN
#define SIM800_POWER_PIN 5
#endif

#ifndef SIM800_TX_PIN
#define SIM800_TX_PIN 6
#endif

#ifndef SIM800_RX_PIN
#define SIM800_RX_PIN 7
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

#ifndef DHT22_POWER_PIN
#define DHT22_POWER_PIN 11
#endif

#ifndef DHT22_DATA_PIN
#define DHT22_DATA_PIN 12
#endif

#ifndef DHT_TYPE
#define DHT_TYPE DHT22
#endif

#ifndef BUTTON_CALIBRATE_PIN
#define BUTTON_CALIBRATE_PIN A0
#endif

#ifndef BUTTON_CALIBRATE_PIN_ACTIVE_STATE
#define BUTTON_CALIBRATE_PIN_ACTIVE_STATE LOW
#endif

#ifndef BUTTON_TARE_PIN
#define BUTTON_TARE_PIN A1
#endif

#ifndef BUTTON_TARE_PIN_ACTIVE_STATE
#define BUTTON_TARE_PIN_ACTIVE_STATE LOW
#endif

#ifndef BUTTON_TIMER_MODE
#define BUTTON_TIMER_MODE A2
#endif