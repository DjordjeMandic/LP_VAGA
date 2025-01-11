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

float calculate_supply_voltage(uint16_t adc_value, uint16_t reference_voltage = 1100);

float get_supply_voltage(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

bool timer_mode_button_pressed_on_boot = false;

void setup()
{
    power_all_off();

    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_PIN_MODE(BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE));

    /* set baud rate in Config.hpp */
    serial_begin();
    Serial.println(F("Starting"));

    builtin_led_on();

    bool calibrate_button_pressed = BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE);
    bool tare_button_pressed      = BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE);
    bool internal_reference_set_button_pressed = BUTTON_PRESSED(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE);
    timer_mode_button_pressed_on_boot = BUTTON_PRESSED(BUTTON_TIMER_MODE_PIN, BUTTON_TIMER_MODE_PIN_ACTIVE_STATE);

    uint8_t button_pressed_count = calibrate_button_pressed + tare_button_pressed + internal_reference_set_button_pressed + timer_mode_button_pressed_on_boot;

    if (button_pressed_count > 1)
    {
        /* multiple buttons pressed, block */
        Serial.println(F("Multiple buttons pressed"));
        show_result_final_block(false);
    }

    /* calibrate internal adc reference */
    if (internal_reference_set_button_pressed)
    {
        /* set internal reference */
        ADCHelper::refBGInit();

        Serial.println(F("Reference calibration: Enter voltage in milivolts on AREF pin."));

        String input = "";
        bool inputComplete = false;
        unsigned long startTime = millis();

        /* wait for user to input the voltage */
        do
        {
            while (Serial.available() > 0)
            {
                char c = Serial.read();
                if (c == '\n') 
                {
                    input.trim();
                    inputComplete = true;

                    // Flush rest of the buffer
                    while (Serial.available() > 0) 
                    {
                        Serial.read();
                    }
                    break;       
                }
                input += c;
            }

            if (millis() - startTime >= 3000)
            {
                Serial.println(F("Enter AREF voltage in milivolts: "));

                startTime = millis();
            }
        } while (!inputComplete);

        /* convert input to long */
        long inputLong = input.toInt();

        Serial.print(F("User input: "));
        Serial.print(inputLong);
        Serial.println(F(" mV"));

        /* set internal reference if valid */
        if ((inputLong < 900 || inputLong > 1300))
        {
            /* save to EEPROM */
            DataEEPROM::setInternalAdcReference(static_cast<uint16_t>(inputLong));

            Serial.print(F("Saved new value: "));
            Serial.println(inputLong);

            /* redo measurement */
            float supply_voltage = get_supply_voltage();

            Serial.print(F("Supply voltage: "));
            Serial.print(supply_voltage, 3);
            Serial.println(F(" V"));

            show_result_final_block(true);
        }

        /* input is invalid */
        show_result_final_block(false);
    }

    /* check avcc voltage, power down if too low for modules to work */
    float supply_voltage = get_supply_voltage();
    if (supply_voltage < AVCC_MIN_VOLTAGE)
    {
        Serial.print(F("AVCC voltage too low: "));
        Serial.print(supply_voltage, 3);
        Serial.print(F(" V, Minimum required: "));
        Serial.print(AVCC_MIN_VOLTAGE, 3);
        Serial.println(F(" V"));

        /* power down with led blinking */
        show_result_final_block(false);
    }

    /* calibrate or tare scale */
    if (calibrate_button_pressed || tare_button_pressed)
    {
        Serial.println(F("Calibrate or tare operation requested"));
        Serial.println(F("HX711 initializing"));

        /* power on scale and init library */
        ScaleModule::begin();
        bool scale_status = true;

        /* wait for scale to be ready */
        scale_status &= ScaleModule::waitReadyTimeout();

        builtin_led_off();
        
        /* perform scale stabilization only if scale is ready */
        if (scale_status)
        {
            Serial.println(F("HX711 stabilizing"));

            /* stabilize scale using dummy readings */
            scale_status &= ScaleModule::stabilize();
        }

        /* perform tare only if scale is stabilized */
        if (scale_status)
        {
            Serial.println(F("HX711 taring"));

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

        /* store to EEPROM */
        DataEEPROM::setScaleTareOffset(new_tare_offset);
        
        Serial.print(F("Saved new tare offset: "));
        Serial.println(new_tare_offset);

        if (tare_button_pressed)
        {
            Serial.println(F("Only tare operation requested"));

            /* operation successful */
            show_result_final_block(true);
        }

        if (calibrate_button_pressed)
        {
            /* calibrate scale */
            Serial.println(F("Calibrating scale"));

            float known_mass_kg = SCALE_CALIBRATION_KNOWN_MASS_KG;

            /* sleep for 2 seconds */
            sleep_power_down_2065ms_adc_off_bod_on();

            /* wait for user to release calibrate button if not released yet */
            sleep_while(SLEEP_MODE_IDLE, BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE));

            /* wait for user to place known weight and press calibrate button */
            do
            {

                Serial.print(F("Press calibrate button after "));
                Serial.print(known_mass_kg, 0);
                Serial.println(F(" kg weight is placed"));

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

            Serial.print(F("Saved scale factor: "));
            Serial.println(new_scale_factor);

            /* block end of calibration */
            show_result_final_block(true);
        }

        /* operation unknown */
        Serial.println(F("Unknown scale operation"));
        show_result_final_block(false);
    }

    Serial.println(F("RTC initializing"));

    /* power on rtc */
    RTCModule::preBeginPowerOn();

    /* wait for rtc to power up then start i2c communication */
    if (!RTCModule::begin())
    {
        /* failed to setup rtc, power down with led blinking */
        show_result_final_block(false);
    }

    if (RTCModule::lostPower())
    {
        /* if timer mode button is not pressed on boot */
        if (!timer_mode_button_pressed_on_boot)
        {
            Serial.println(F("Power loss detected. Set the clock or use timer mode"));

            /* power down with led blinking */
            show_result_final_block(false);
        }

        /* timer mode button is pressed on boot */

        Serial.println(F("Adjusting time to 2025-01-01 00:00:00"));

        DateTime dt = DateTime(2025, 1, 1, 0, 0, 0);

        if (!RTCModule::adjust(dt))
        {
            /* failed to setup rtc, power down with led blinking */
            show_result_final_block(false);
        }
    }
}

void loop()
{
    /* loop is only for low power optimized operation */
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


/**
 * @brief Calculate the microcontroller's supply voltage (AVcc) using the internal 1.1V reference.
 * 
 * This function initializes the ADC to measure the internal 1.1V bandgap reference against the
 * supply voltage (AVcc). It calculates the supply voltage by taking multiple samples, averaging 
 * the ADC readings, and using a known reference voltage stored in EEPROM.
 * 
 * @param[in] samples The number of ADC samples to average for better accuracy.
 * @return The calculated supply voltage (AVcc) in volts.
 * 
 * Steps:
 * - Initializes the ADC for AVcc as the reference.
 * - Takes the specified number of ADC samples of the internal 1.1V reference and averages them.
 * - Retrieves the calibrated 1.1V reference voltage (in millivolts) from EEPROM.
 * - Calculates and returns the supply voltage using the averaged ADC reading and the known reference voltage.
 */
float get_supply_voltage(uint8_t samples)
{
    ADCHelper::avccInit();

    /* get average adc reading of internal reference against avcc */
    uint16_t adc_reading = ADCHelper::avccSampleAverage(samples);

    ADCHelper::end();

    uint16_t reference_voltage;

    DataEEPROM::getInternalAdcReference(reference_voltage);

    return calculate_supply_voltage(adc_reading, reference_voltage);
}

/**
 * @brief Calculate the supply voltage based on ADC measurement of the internal 1.1V reference.
 * 
 * This function calculates the microcontroller's supply voltage when the supply voltage is used 
 * as the ADC reference and the internal 1.1V bandgap is being measured.
 * 
 * @param adc_value ADC value when measuring the internal 1.1V bandgap reference.
 * @param bandgap_voltage_mV The measured voltage of the internal 1.1V reference in millivolts (default: 1100 mV).
 * @return The calculated supply voltage in volts.
 */
float calculate_supply_voltage(uint16_t adc_value, uint16_t bandgap_voltage_mV = 1100)
{
    /* Calculate supply voltage, supply voltage is used as reference while sampling 1.1v bandgap */
    uint32_t supply_voltage_milivolts = (1024 * bandgap_voltage_mV);
    supply_voltage_milivolts += (adc_value / 2U);
    supply_voltage_milivolts /= adc_value;

    return static_cast<float>(supply_voltage_milivolts) / 1000.0f;
}