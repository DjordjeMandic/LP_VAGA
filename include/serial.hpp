#pragma once

#include <Arduino.h>
#include <Config.hpp>

#define serial_print(x) { if (serial_is_enabled()) { Serial.print(x); } }
#define serial_println(x) { if (serial_is_enabled()) { Serial.println(x); } }
#define serial_flush() { if (serial_is_enabled()) { Serial.flush(); } }

void serial_begin(unsigned long baudrate = SERIAL_BAUD);
void serial_end();

bool serial_is_enabled();