#pragma once

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define	sleep_while(mode, condition)	\
do { 						\
      set_sleep_mode(mode); \
      cli();				\
      sleep_enable();		\
      sei();				\
      sleep_cpu();			\
      sleep_disable();		\
      sei();				\
} while (condition)

#define sleep_until(mode, condition)    \
    sleep_while(mode, !(condition))

void sleep_timeout_millis(uint8_t mode, unsigned long sleep_time_ms);
