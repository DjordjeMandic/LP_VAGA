#include <Arduino.h>
#include <SPI.h>
#include <LowPower.h>
#include <HX711.h>
#include <power/hx711.hpp>
#include <power/ds3231.hpp>
#include <power/gsm.hpp>
#include <power/dht22.hpp>
#include <module/scale.hpp>
#include <module/dht.hpp>
#include <Serial.hpp>
#include <RTClib.h>
#include <DHT.h>


#define BUTTON_PIN_MODE(BUTTON_ACTIVE_STATE) (BUTTON_ACTIVE_STATE == LOW ? INPUT_PULLUP : INPUT)
#define BUTTON_PRESSED(BUTTON_PIN, BUTTON_ACTIVE_STATE) (digitalRead(BUTTON_PIN) == BUTTON_ACTIVE_STATE)

// create function for calibration
// create function for tare

// create rtc module

// todo use WDT sleep delay for unimportant delays

bool user_scale_tare();
bool user_scale_calibration();


void setup()
{
    gsm_power_off();
    scale_end();
    dht_end();
    ds3231_power_off();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));

    if (BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE) ||
        BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE))
    {
        /* power on scale and init library */
        scale_begin();
        bool scale_status = true;

        /* wait for scale to be ready, power on delay + 1000ms for first sample */
        scale_status &= scale_wait_ready_timeout(HX711_POWER_DELAY_MS + 1000, 100);
        
        /* stabilize scale using dummy readings for 3 seconds */
        scale_status &= scale_stabilize(3000);

    }

    scale_begin();
    serial_enable();
    serial_println(F("Start..."));
    while(!hx711_power_delay_check())
    {
        serial_println(F("Waiting for HX711 to power on..."));
    }
    if (!scale_wait_ready_timeout(1000))
    {
        while (1)
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }
}

void loop()
{
    for(uint8_t i = 0; i < 100; i++) {
        if (scale_ready()) {
            long reading = scale_read();
            Serial.print("HX711 reading: ");
            Serial.println(reading);
        } else {
            Serial.println("Not ready");
        }
        delay(100);
    }

    scale_end();
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

bool user_scale_tare()
{

}

bool user_scale_calibration()
{



    return true;
}