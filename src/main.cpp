#include <Arduino.h>
#include <SPI.h>
#include <LowPower.h>
#include <HX711.h>
#include <power/HX711PowerManager.hpp>
#include <power/DS3231PowerManager.hpp>
#include <power/SIM800PowerManager.hpp>
#include <power/DHT22PowerManager.hpp>
#include <module/scale.hpp>
#include <module/dht.hpp>
#include <module/rtc.hpp>
#include <power/ADCHelper.hpp>
#include <Serial.hpp>
#include <RTClib.h>
#include <DHT.h>


#define BUTTON_PIN_MODE(BUTTON_ACTIVE_STATE) (BUTTON_ACTIVE_STATE == LOW ? INPUT_PULLUP : INPUT)
#define BUTTON_PRESSED(BUTTON_PIN, BUTTON_ACTIVE_STATE) (digitalRead(BUTTON_PIN) == BUTTON_ACTIVE_STATE)

#define builtin_led_on() { digitalWrite(LED_BUILTIN, HIGH); }
#define builtin_led_off() { digitalWrite(LED_BUILTIN, LOW); }
#define builtin_led_toggle() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }


// create function for calibration
// create function for tare

// create rtc module

// todo use WDT sleep delay for unimportant delays

bool user_scale_tare();
bool user_scale_calibration();

void power_all_off();

void show_result_final_block(bool success = false);

void setup()
{
    power_all_off();

    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));

    bool calibrate_button_pressed = BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE);
    bool tare_button_pressed      = BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE);

    uint8_t button_pressed_count = calibrate_button_pressed + tare_button_pressed;

    /* set baud rate in Config.hpp */
    serial_begin();

    if (button_pressed_count > 1)
    {
        /* both buttons pressed, block */
        serial_println(F("Multiple buttons pressed"));
        show_result_final_block(false);
    }

    if (calibrate_button_pressed || tare_button_pressed)
    {
        serial_println(F("HX711 initializing"));
        
        /* power on scale and init library */
        scale_begin();
        bool scale_status = true;

        /* wait for scale to be ready, power on delay + 1000ms for first sample */
        scale_status &= scale_wait_ready_timeout(HX711_POWER_DELAY_MS + 1000);
        
        /* perform scale stabilization only if scale is ready */
        if (scale_status)
        {
            serial_println(F("HX711 stabilizing"));
            /* stabilize scale using dummy readings for 3 seconds */
            scale_status &= scale_stabilize(SCALE_STABILIZATION_TIME_MS);
        }

        if (!scale_status)
        {
            /* failed to setup scale, block */
            while (1);
        }

        if (BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE))
        {
            /* tare scale */

            /* block end of tare */
            while (1);
        }
        else if (BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE))
        {
            /* calibrate scale */

            /* block end of calibration */
            while (1);
        }
    }

    scale_begin();
    serial_begin();
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
        /* Clock startup time is around 67ms, this gives delay of around 120ms */
        /* TODO: test this */
        LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_ON);
    }
    delay(100);
    scale_end();
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void power_all_off()
{
    /* todo replace this with gsm_end */
    sim800_power_off();

    /* disable physical sensors */
    scale_end();
    dht_end();

    /* disable rtc and twi peripheral */
    rtc_end();

    /* disable on chip uart and adc pheripherals */
    serial_end();
    adc_end();

    /* disable spi and unnecessary timers */
    power_spi_disable();
    power_timer1_disable();
    power_timer2_disable();

    /* disable builtin led */
    pinMode(LED_BUILTIN, OUTPUT);
    builtin_led_off();
}

bool user_scale_tare()
{

}

bool user_scale_calibration()
{



    return true;
}

void show_result_final_block(bool success)
{
    serial_print(F("Result: "));
    serial_print(success ? F("ok") : F("fail"));
    serial_println(F(" | Press reset to reboot"));

    power_all_off();

    detachInterrupt(digitalPinToInterrupt(RTC_INT_PIN));
    detachInterrupt(digitalPinToInterrupt(ACCEL_INT_PIN));

    if (success)
    {
        /* Done successfully, turn on builtin led, sleep forever */
        builtin_led_on();

        do
        {
            LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
        } while (true);
    }

    /* Failed, blink builtin led forever */
    do
    {
        builtin_led_on();
        LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_ON);
        builtin_led_off();
        LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_ON);
    } while (true);
}