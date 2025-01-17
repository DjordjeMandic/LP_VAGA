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
#include <Strings_EN.h>
#include <module/GSMModule.hpp>


#define BUTTON_PIN_MODE(BUTTON_ACTIVE_STATE) (BUTTON_ACTIVE_STATE == LOW ? INPUT_PULLUP : INPUT)
#define BUTTON_PRESSED(BUTTON_PIN, BUTTON_ACTIVE_STATE) (digitalRead(BUTTON_PIN) == BUTTON_ACTIVE_STATE)

#define builtin_led_on() { digitalWrite(LED_BUILTIN, HIGH); }
#define builtin_led_off() { digitalWrite(LED_BUILTIN, LOW); }
#define builtin_led_toggle() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }

#define RESULT_SUCCESS true
#define RESULT_FAILURE false


// create function for calibration
// create function for tare

// create rtc module

// todo use WDT sleep delay for unimportant delays

void power_all_off();

void show_setup_result_final_block(bool success = false);
void show_setup_result_serial_only(bool success = false);

float get_supply_voltage(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

bool timer_mode_button_pressed_on_boot = false;

const DateTime timer_mode_datetime = DateTime(2025, 1, 1, 0, 0, 0);

bool setup_calibrate_internal_reference();
bool setup_calibrate_scale_factor();

/* sms buffer, maximum length is 160 chars */
char sms_buffer[161] = {0};

/* use with snpringf_P */
const char flashString[] PROGMEM = "";

uint16_t internal_reference = DataEEPROM::getInternalAdcReference();
float scale_factor = DataEEPROM::getScaleCalibrationValue();
long scale_tare_offset = DataEEPROM::getScaleTareOffset();


float calculate_supply_voltage(uint16_t adc_value, uint16_t reference_voltage = internal_reference);

void setup()
{
    power_all_off();
    
    /* alive signal */
    builtin_led_on();

    GSMModule::begin();
    GSMModule::registeredOnNetwork();

    /* configure buttons */
    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_PIN_MODE(BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TIMER_MODE_PIN, BUTTON_PIN_MODE(BUTTON_TIMER_MODE_PIN_ACTIVE_STATE));

    /* set baud rate in Config.hpp */
    serial_begin();
    Serial.println(F(STRING_STARTUP_MESSAGE));

    /* read buttons */
    bool calibrate_button_pressed = BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE);
    bool tare_button_pressed      = BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE);
    bool internal_reference_set_button_pressed = BUTTON_PRESSED(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE);
    timer_mode_button_pressed_on_boot = BUTTON_PRESSED(BUTTON_TIMER_MODE_PIN, BUTTON_TIMER_MODE_PIN_ACTIVE_STATE);
    /* count number of pressed buttons */
    uint8_t button_pressed_count = calibrate_button_pressed + tare_button_pressed + internal_reference_set_button_pressed + timer_mode_button_pressed_on_boot;
    /* if multiple buttons are pressed, block */
    if (button_pressed_count > 1)
    {
        Serial.println(F(STRING_MULTIPLE_BUTTONS_PRESSED));
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* calibrate internal adc reference if requested */
    if (internal_reference_set_button_pressed)
    {
        builtin_led_off();
        show_setup_result_final_block(setup_calibrate_internal_reference());
    }

    /* check internal reference */
    if (internal_reference < 1000 || internal_reference > 1200)
    {
        Serial.println(F(STRING_REF_INVALID));
        show_setup_result_final_block(RESULT_FAILURE);
    }
    /* check min avcc voltage constant */
    static_assert(float(AVCC_MIN_VOLTAGE) >= 3.5f, "AVCC_MIN_VOLTAGE must be above 3.5V for modules to work");
    static_assert(float(AVCC_MIN_VOLTAGE) < 4.2f, "AVCC_MIN_VOLTAGE must be below 4.2V");
    float supply_voltage = get_supply_voltage(); /* read supply voltage */
    Serial.print(F("AVCC: "));
    Serial.print(supply_voltage, 3);
    Serial.print(F(" V ; MIN: "));
    Serial.print(AVCC_MIN_VOLTAGE, 3);
    Serial.print(F(" V, REF: "));
    Serial.print(internal_reference, 3);
    Serial.println(F(" mV"));
    /* check avcc voltage, power down if too low for modules to work */
    if (supply_voltage < AVCC_MIN_VOLTAGE)
    {
        /* power down with led blinking */
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* test hx711 */
    Serial.println(F(STRING_SCALE_INITIALIZING));
    ScaleModule::begin(); /* power on scale and init library */
    /* apply scale parameters */
    ScaleModule::setOffset(scale_tare_offset);
    ScaleModule::setScale(scale_factor);
    Serial.print(F(STRING_TARE_OFFSET_SC_SPACE));
    Serial.println(scale_tare_offset);
    Serial.print(F(STRING_SCALE_FACTOR_SC_SPACE));
    Serial.println(scale_factor, 5);
    bool scale_status = true;
    builtin_led_on(); /* indicate start of procedure */
    scale_status &= ScaleModule::waitReadyTimeout(); /* wait for scale to be ready */
    /* perform scale stabilization only if scale is ready */
    if (scale_status)
    {
        Serial.println(F(STRING_SCALE_STABILIZING));
        scale_status &= ScaleModule::stabilize(); /* stabilize scale using dummy readings */
    }
    builtin_led_off();
    show_setup_result_serial_only(scale_status);
    /* if scale is stabilized, test */
    if (scale_status)
    {
        /* Measure average 10x */
        Serial.print(F(STRING_MEASURED_MASS_SC_SPACE));
        Serial.print(ScaleModule::getUnits(10), 3);
        Serial.println(F(STRING_SPACE_KG));

        /* calibrate or tare scale if requested */
        if (calibrate_button_pressed || tare_button_pressed)
        {
            Serial.println(F(STRING_SCALE_TARING));
            scale_status &= ScaleModule::tare(); /* tare operation is required anyways for calibration */
            /* if failed to tare scale, block */
            if (!scale_status)
            {
                show_setup_result_final_block(RESULT_FAILURE);
            }
            scale_tare_offset = ScaleModule::getOffset(); /* get new tare offset */
            DataEEPROM::setScaleTareOffset(scale_tare_offset); /* store to EEPROM */
            Serial.print(F(STRING_SAVED_TARE_OFFSET_SC_SPACE));
            Serial.println(scale_tare_offset);

            /* only tare operation requested */
            if (tare_button_pressed)
            {
                /* operation successful */
                show_setup_result_final_block(RESULT_SUCCESS);
            }

            /* calibrate scale */
            if (calibrate_button_pressed)
            {
                /* block end of calibration */
                show_setup_result_final_block(setup_calibrate_scale_factor());
            }

            /* operation unknown */
            Serial.println(F(STRING_UNKNOWN_COMMAND));
            show_setup_result_final_block(RESULT_FAILURE);
        }
    }

    /* test DHT */
    Serial.println(F(STRING_DHT_INITIALIZING));
    DHTModule::begin(); /* initialize DHT sensor */
    builtin_led_on(); /* indicate start of procedure */
    sleep_until(SLEEP_MODE_IDLE, DHT22PowerManager::poweredOn()); /* wait for DHT sensor to power up */
    builtin_led_off();
    bool dht_status = DHTModule::ready(); /* check if DHT sensor is ready */
    float temp = DHTModule::readTemperature();
    float humidity = DHTModule::readHumidity();
    Serial.print(F("Temperature: "));
    Serial.print(temp, 1);
    Serial.print(F(" C ; Humidity: "));
    Serial.print(humidity, 1);
    Serial.println(F(" %"));
    /* check if temperature and humidity are finite */
    dht_status &= isfinite(temp);
    dht_status &= isfinite(humidity);
    show_setup_result_serial_only(dht_status);

    /* test RTC */
    Serial.println(F(STRING_RTC_INITIALIZING));
    RTCModule::preBeginPowerOn(); /* power on rtc */
    builtin_led_on(); /* indicate start of procedure */
    bool rtc_status = false; /* assume rtc is not ready */
    bool rtc_lost_power = true; /* assume power loss */
    bool rtc_time_valid = false; /* assume time is not valid */
    bool rtc_timer_mode_time_adjusted = false; /* assume timer time has not been adjusted */
    DateTime date_time;

    do
    {
        rtc_status = RTCModule::begin(); /* wait for rtc to power up then start i2c communication */
        builtin_led_off();

        /* if rtc is not ready then break */
        if (!rtc_status)
        {
            break;
        }

        /* check for power loss */
        rtc_lost_power = RTCModule::lostPower();
        if (rtc_lost_power)
        {
            Serial.println(F(STRING_RTC_LOST_POWER));
            if (!timer_mode_button_pressed_on_boot)
            {
                Serial.println(F(STRING_SET_CLOCK_OR_USE_TIMER_MODE));
                break; /* time is not valid and timer mode is not enabled */

            }
            Serial.println(F(STRING_TIMER_ENABLED_SETTING_TIME));
            rtc_timer_mode_time_adjusted = RTCModule::adjust(timer_mode_datetime);
            if (!rtc_timer_mode_time_adjusted)
            {
                break;
            }
        }

        Serial.println(F(STRING_READ_RTC_TIME));
        date_time = RTCModule::now(); /* read time from rtc */
        rtc_time_valid = date_time.isValid(); /* check if rtc time is valid */
        /* if rtc time is valid print it */
        if (rtc_time_valid)
        {
            /* print time */
            Serial.printf(
                F(STRING_RTC_TIME_SC_SPACE "%02u/%02u/%04u %02u:%02u:%02u\n"),
                date_time.day(),     // DD
                date_time.month(),   // MM
                date_time.year(),    // YYYY
                date_time.hour(),    // HH
                date_time.minute(),  // MM
                date_time.second()   // SS
            );
        }
        rtc_time_valid &= date_time >= timer_mode_datetime; /* check if rtc time is greater than or equal to timer mode time */
        if (!rtc_time_valid)
        {
            Serial.println(F(STRING_RTC_TIME_INVALID));
        }
    } while (false);
    show_setup_result_serial_only(rtc_time_valid);

    /* test GSM, if test passes, send report */


    // dht_status - true if DHT passed the test
    // scale_status - true if scale passed the test
    // rtc_status - true if rtc is initialized
    // rtc_lost_power - true if rtc lost power
    // timer_mode_time_adjusted - true if timer mode time is set
    // time_valid - true if time is valid


    /* test failed, display result and block, do not continue to loop */
    show_setup_result_final_block(RESULT_FAILURE);
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

void show_setup_result_serial_only(bool success)
{
    if (serial_is_enabled())
    {
        Serial.print(F(STRING_OPERATION_RESULT_SC_SPACE));
        Serial.println(success ? F(STRING_OK) : F(STRING_FAIL));
    }
}

void show_setup_result_final_block(bool success)
{
    show_setup_result_serial_only(success);
    serial_println(F(STRING_HALTED));

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


float get_supply_voltage(uint8_t samples)
{
    ADCHelper::avccInit();

    /* get average adc reading of internal reference against avcc */
    uint16_t adc_reading = ADCHelper::avccSampleAverage(samples);
    
    ADCHelper::end();

    return calculate_supply_voltage(adc_reading, internal_reference);
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
float calculate_supply_voltage(uint16_t adc_value, uint16_t bandgap_voltage_mV)
{
    /* Calculate supply voltage, supply voltage is used as reference while sampling 1.1v bandgap */
    uint32_t supply_voltage_milivolts = static_cast<uint32_t>((1 << 10) - 1) * bandgap_voltage_mV;
    supply_voltage_milivolts += (adc_value / 2U);
    supply_voltage_milivolts /= adc_value;
    return (supply_voltage_milivolts * 1.0F) / 1000.0f;
}

bool setup_calibrate_internal_reference()
{
    /* initialize adc for internal reference calibration */
    ADCHelper::refBGInit();

    Serial.println(F(STRING_MEASURE_VOLTAGE_ON_AREF_PIN));

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

                /* flush rest of the buffer */
                while (Serial.available() > 0) 
                {
                    Serial.read();
                }
                break;       
            }
            input += c;
        }

        /* heartbeat led and serial prompt */
        if (millis() - startTime >= 3000)
        {
            Serial.println(F(STRING_ENTER_AREF_IN_MV));
            startTime = millis();

            builtin_led_on();
            sleep_idle_timeout_millis(100);
            builtin_led_off();
        }
    } while (!inputComplete);

    /* convert input to long */
    long inputLong = input.toInt();

    Serial.print(F(STRING_INPUT_SC_SAPCE));
    Serial.println(inputLong);

    /* set internal reference if within range */
    if ((inputLong > 1000 || inputLong < 1200))
    {
        /* save to EEPROM */
        internal_reference = static_cast<uint16_t>(inputLong);
        DataEEPROM::setInternalAdcReference(internal_reference);

        Serial.print(F(STRING_SAVED_SC_SAPCE));
        Serial.println(inputLong);

        /* redo measurement */
        Serial.print(F(STRING_SUPPLY_VOLTAGE_SC_SPACE));
        Serial.print(get_supply_voltage(), 3);
        Serial.println(F(STRING_SPACE_V));

        return RESULT_SUCCESS;
    }

    /* input is invalid */
    return RESULT_FAILURE;
}

bool setup_calibrate_scale_factor()
{
    static_assert(float(SCALE_CALIBRATION_KNOWN_MASS_KG) != 0.0f, "Known mass must be non-zero.");

    float known_mass_kg = SCALE_CALIBRATION_KNOWN_MASS_KG;

    if (!isfinite(known_mass_kg))
    {
        Serial.println(F(STRING_KNOWN_MASS_NOT_FINITE));
        return RESULT_FAILURE;
    }

    /* wait for user to place known weight and press calibrate button */
    do
    {
        Serial.print(F(STRING_PLACE_SPACE));
        Serial.print(known_mass_kg, 0);
        Serial.println(F(STRING_SPACE_KG_ON_SCALE_AND_PRESS_TARE));

        builtin_led_on();
        sleep_power_down_185ms_adc_off_bod_on();
        builtin_led_off();
        sleep_power_down_1065ms_adc_off_bod_on();
    } while (!BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE));

    /* set scale factor to 1 (default) */
    ScaleModule::setScale(1.0f);

    /* measure average 10x and calculate new scale factor */
    scale_factor = ScaleModule::getUnits(10) / known_mass_kg;

    if (!isfinite(scale_factor) || scale_factor == 0.0f)
    {
        Serial.println(F(STRING_NEW_SCALE_FACTOR_INVALID));
        return RESULT_FAILURE;
    }

    /* save new scale factor to EEPROM */
    DataEEPROM::setScaleCalibrationValue(scale_factor);

    Serial.print(F(STRING_SAVED_FACTOR_SC_SPACE));
    Serial.println(scale_factor);

    return RESULT_SUCCESS;
}
