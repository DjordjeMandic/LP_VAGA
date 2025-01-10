#pragma once

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/**
 * @brief Macro to put the microcontroller into sleep mode while a specified condition is true.
 * 
 * This macro sets the desired sleep mode, disables interrupts, enables sleep, re-enables interrupts,
 * and puts the microcontroller to sleep. It continues looping in sleep mode until the specified 
 * condition becomes false.
 * 
 * In `SLEEP_MODE_IDLE`, Timer0 continues to run if it is enabled, and its overflow interrupt 
 * is fired approximately every 1024 microseconds (~1ms).
 * 
 * @param[in] mode The desired sleep mode (e.g., `SLEEP_MODE_IDLE`, `SLEEP_MODE_ADC`).
 * @param[in] condition The condition to continue sleeping. The loop will exit when this condition becomes false.
 */

#define sleep_while(mode, condition)    \
do {                                    \
      set_sleep_mode(mode);             \
      cli();                            \
      sleep_enable();                   \
      sei();                            \
      sleep_cpu();                      \
      sleep_disable();                  \
      sei();                            \
} while (condition)

/**
 * @brief Macro to put the microcontroller into sleep mode until a specified condition is true.
 * 
 * This macro is similar to `sleep_while` but negates the condition. It puts the microcontroller
 * into sleep mode until the specified condition becomes true.
 * 
 * @param[in] mode The desired sleep mode (e.g., `SLEEP_MODE_IDLE`, `SLEEP_MODE_ADC`).
 * @param[in] condition The condition to wake up. The loop will exit when this condition becomes true.
 */
#define sleep_until(mode, condition)    \
    sleep_while(mode, !(condition))

/**
 * @brief Puts the microcontroller into `IDLE` sleep mode for a specified duration in milliseconds.
 * 
 * This function uses `SLEEP_MODE_IDLE`, where the CPU is halted, but Timer0 continues to run.
 * Since Timer0 remains active, functions like `millis()` and `delay()` continue working normally,
 * allowing the sleep duration to be tracked using `millis()`.
 * 
 * @param sleep_time_ms The duration to sleep, in milliseconds.
 */
void sleep_idle_timeout_millis(unsigned long sleep_time_ms);

/**
 * @brief Sleep for 80ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (15ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_80ms_adc_off_bod_on();

/**
 * @brief Sleep for 95ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (30ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_95ms_adc_off_bod_on();

/**
 * @brief Sleep for 125ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (60ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_125ms_adc_off_bod_on();

/**
 * @brief Sleep for 185ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (120ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_185ms_adc_off_bod_on();

/**
 * @brief Sleep for 315ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (250ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_315ms_adc_off_bod_on();

/**
 * @brief Sleep for 565ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (500ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_565ms_adc_off_bod_on();

/**
 * @brief Sleep for 1065ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (1000ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_1065ms_adc_off_bod_on();

/**
 * @brief Sleep for 2065ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (2000ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_2065ms_adc_off_bod_on();

/**
 * @brief Sleep for 4065ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (4000ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_4065ms_adc_off_bod_on();

/**
 * @brief Sleep for 8065ms in power-down mode with ADC off and BOD on.
 * 
 * This includes 65ms startup time for the external full-swing crystal and 
 * the remaining time (8000ms) is the WDT timeout, which is inaccurate.
 * During this time, Timer0 is not running, so `millis()` and `micros()` 
 * will not be updated. Before entering sleep, this function calls 
 * `serial_flush()` to check if the serial is enabled and flushes the 
 * serial buffer if necessary.
 */
void sleep_power_down_8065ms_adc_off_bod_on();
