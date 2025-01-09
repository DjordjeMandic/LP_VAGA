#pragma once

#include <Arduino.h>
#include <Config.hpp>

#define serial_print(x) { if (serial_is_enabled()) { Serial.print(x); } }
#define serial_println(x) { if (serial_is_enabled()) { Serial.println(x); } }

void serial_enable(unsigned long baudrate = SERIAL_BAUD);
void serial_disable();

bool serial_is_enabled();