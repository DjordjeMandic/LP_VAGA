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
