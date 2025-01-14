#pragma once

#include <Arduino.h>

#ifndef SCALE_CALIBRATION_KNOWN_MASS_KG
#define SCALE_CALIBRATION_KNOWN_MASS_KG 2
#endif

#ifndef AVCC_MIN_VOLTAGE
#define AVCC_MIN_VOLTAGE 3.5 /* sim800 has undervoltage warning below 3.5 */
#endif

#ifndef ADC_AVCC_SAMPLES_DEFAULT
#define ADC_AVCC_SAMPLES_DEFAULT 10
#endif

#ifndef ADC_AVCC_REF_STABILIZATION_DELAY_MS
#define ADC_AVCC_REF_STABILIZATION_DELAY_MS 15
#endif

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

#ifndef BUTTON_TIMER_MODE_PIN
#define BUTTON_TIMER_MODE_PIN A2
#endif

#ifndef BUTTON_TIMER_MODE_PIN_ACTIVE_STATE
#define BUTTON_TIMER_MODE_PIN_ACTIVE_STATE LOW
#endif

#ifndef BUTTON_INTERNAL_REFERENCE_SET_PIN
#define BUTTON_INTERNAL_REFERENCE_SET_PIN A3
#endif

#ifndef BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE
#define BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE LOW
#endif