#pragma once

#include <Arduino.h>
#include <Config.hpp>
#include <power/SIM800PowerManager.hpp>

#define _SS_MAX_RX_BUFF 256
#include <SoftwareSerial.h>

#define gsm_return_if_not_powered_on(x) { if (!SIM800PowerManager::powered_on()) { return x; } }
#define gsm_return_if_not_ready(x) { if (!gsm_ready()) { return x; } }

static_assert(_SS_MAX_RX_BUFF >= 256, "SS_MAX_RX_BUFF must be at least 256.");

// VBAT goes high, wait 1.4 seconds
// VDD goes high, wait 3 seconds, UART ready


class GSMModule
{
public:
    static void preBeginPowerOn();

    static bool begin(unsigned long current_millis = millis());

    static bool end();

    static Stream& getStream();
private:
    static bool ready_; /**< Stores the readiness state of the GSM Module. */
    static SoftwareSerial* software_serial_;
};
