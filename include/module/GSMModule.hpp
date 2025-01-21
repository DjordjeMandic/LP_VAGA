#pragma once

#include <Arduino.h>
#include <Config.hpp>
#include <power/SIM800PowerManager.hpp>
#include <SoftwareSerial.h>

#define gsm_return_if_not_powered_on(x) { if (!SIM800PowerManager::powered_on()) { return x; } }
#define gsm_return_if_not_ready(x) { if (!gsm_ready()) { return x; } }


/* casts to const __FlashStringHelper pointer */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))

/* casts to PGM_P */
#define FP(fsh_pointer) (reinterpret_cast<PGM_P>(fsh_pointer))


#define GSM_COMMAND_MAX_LEN 550

#define GSM_NUMBER_TEXT_MAX_LEN 15
#define GSM_SMS_TEXT_MAX_LEN 159

// VBAT goes high, wait 1.4 seconds
// VDD goes high, wait 3 seconds, UART ready


class GSMModule
{
public:
    static void preBeginPowerOn();

    static bool begin(unsigned long current_millis = millis(), bool save_baud_rate = true);

    static void end();

    static bool registeredOnNetwork();

    static bool textSMSFormat();

    static bool signalQuality(uint8_t& rssi, uint8_t& ber);

    static bool sendSMS(const char* number, const char* message);

    static Stream& getStream();
private:
    static bool ready_; /**< Stores the readiness state of the GSM Module. */
    static SoftwareSerial* software_serial_;
};
