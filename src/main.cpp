#include <Arduino.h>
#include <SPI.h>
#include <LowPower.h>
#include <HX711.h>
#include <power/HX711PowerManager.hpp>
#include <power/DS3231PowerManager.hpp>
#include <power/SIM800PowerManager.hpp>
#include <power/DHT22PowerManager.hpp>
#include <module/ScaleModule.hpp>
#include <module/DHTModule.hpp>
#include <module/RTCModule.hpp>
#include <power/ADCHelper.hpp>
#include <power/sleep.hpp>
#include <DataEEPROM.hpp>
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

void power_all_off();

void show_result_final_block(bool success = false);

void setup()
{
    power_all_off();

    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));

    /* set baud rate in Config.hpp */
    serial_begin();
    serial_println(F("Starting"));

    builtin_led_on();

    bool calibrate_button_pressed = BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE);
    bool tare_button_pressed      = BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE);

    uint8_t button_pressed_count = calibrate_button_pressed + tare_button_pressed;

    if (button_pressed_count > 1)
    {
        /* multiple buttons pressed, block */
        serial_println(F("Multiple buttons pressed"));
        show_result_final_block(false);
    }

    /* calibrate internal adc reference */

    //todo

    /* check avcc voltage, power down if too low for modules to work */

    //todo

    /* calibrate or tare scale */
    if (calibrate_button_pressed || tare_button_pressed)
    {
        if (serial_is_enabled())
        {
            Serial.println(F("Calibrate or tare operation requested"));
            Serial.println(F("HX711 initializing"));
        }

        /* power on scale and init library */
        ScaleModule::begin();
        bool scale_status = true;

        /* wait for scale to be ready */
        scale_status &= ScaleModule::waitReadyTimeout();

        builtin_led_off();
        
        /* perform scale stabilization only if scale is ready */
        if (scale_status)
        {
            serial_println(F("HX711 stabilizing"));

            /* stabilize scale using dummy readings */
            scale_status &= ScaleModule::stabilize();
        }

        /* perform tare only if scale is stabilized */
        if (scale_status)
        {
            serial_println(F("HX711 taring"));

            /* tare operation is required anyways for calibration */
            scale_status &= ScaleModule::tare();
        }

        if (!scale_status)
        {
            /* failed to setup scale, block */
            show_result_final_block(false);
        }

        /* save new tare offset */
        long new_tare_offset = ScaleModule::getOffset();

        if (serial_is_enabled())
        {
            Serial.print(F("Saving new tare offset: "));
            Serial.println(new_tare_offset);
        }

        /* store to EEPROM */
        DataEEPROM::setScaleTareOffset(new_tare_offset);

        if (tare_button_pressed)
        {
            serial_println(F("Only tare operation requested"));

            /* operation successful */
            show_result_final_block(true);
        }

        if (calibrate_button_pressed)
        {
            /* calibrate scale */
            serial_println(F("Calibrating scale"));

            float known_mass_kg = SCALE_CALIBRATION_KNOWN_MASS_KG;

            /* sleep for 2 seconds */
            sleep_power_down_2065ms_adc_off_bod_on();

            /* wait for user to release calibrate button if not released yet */
            sleep_while(SLEEP_MODE_IDLE, BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE));

            /* wait for user to place known weight and press calibrate button */
            do
            {
                if (serial_is_enabled())
                {
                    Serial.print(F("Press calibrate button after "));
                    Serial.print(known_mass_kg, 0);
                    Serial.println(F("kg weight is placed"));
                }

                builtin_led_on();
                sleep_power_down_185ms_adc_off_bod_on();
                builtin_led_off();
                sleep_power_down_1065ms_adc_off_bod_on();
            } while (!BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
            
            // Call setScale() with no parameter.
            // Call tare() with no parameter.
            // Place a known weight on the scale and call getUnits(10).
            // Divide the result in step 3 to your known weight. You should get about the parameter you need to pass to setScale().
            // Adjust the parameter in step 4 until you get an accurate reading.

            /* set scale factor to 1 */
            ScaleModule::setScale(1.0f);

            float new_scale_factor = ScaleModule::getUnits(10) / known_mass_kg;

            DataEEPROM::setScaleCalibrationValue(new_scale_factor);

            serial_print(F("New scale factor: "));
            serial_println(new_scale_factor);

            /* block end of calibration */
            show_result_final_block(true);
        }

        /* operation unknown */
        serial_println(F("Unknown scale operation"));
        show_result_final_block(false);
    }
}

void loop()
{

}

void power_all_off()
{
    /* todo replace this with GSMModule */
    SIM800PowerManager::powerOff();

    /* disable physical sensors */
    ScaleModule::end();
    DHTModule::end();

    /* disable rtc and twi peripheral */
    RTCModule::end();

    /* disable on chip uart and adc pheripherals */
    serial_end();
    ADCHelper::end();

    /* disable spi and unnecessary timers */
    power_spi_disable();
    power_timer1_disable();
    power_timer2_disable();

    /* disable builtin led */
    pinMode(LED_BUILTIN, OUTPUT);
    builtin_led_off();
}

void show_result_final_block(bool success)
{
    if (serial_is_enabled())
    {
        Serial.print(F("Operation result: "));
        Serial.println(success ? F("ok") : F("fail"));
        Serial.println(F("Press reset to reboot"));
    }

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
        sleep_power_down_80ms_adc_off_bod_on();

        builtin_led_off();
        sleep_power_down_185ms_adc_off_bod_on();
    } while (true);
}