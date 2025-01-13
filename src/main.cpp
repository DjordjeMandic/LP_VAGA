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

#define RESULT_SUCCESS true
#define RESULT_FAILURE false


// create function for calibration
// create function for tare

// create rtc module

// todo use WDT sleep delay for unimportant delays

void power_all_off();

void show_setup_result_final_block(bool success = false);

float calculate_supply_voltage(uint16_t adc_value, uint16_t reference_voltage = 1100);

float get_supply_voltage(uint8_t samples = ADC_AVCC_SAMPLES_DEFAULT);

bool timer_mode_button_pressed_on_boot = false;

const DateTime timer_mode_datetime = DateTime(2025, 1, 1, 0, 0, 0);

bool setup_calibrate_internal_reference();
void setup_calibrate_scale_factor();
void setup_scale_measure();

/* sms buffer, maximum length is 160 chars */
char sms_buffer[161] = {0};

/* use with snpringf_P */
const char flashString[] PROGMEM = "";

void setup()
{
    power_all_off();
    /* alive signal */
    builtin_led_on();
    //snprintf_P
    /* configure buttons */
    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_PIN_MODE(BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TIMER_MODE_PIN, BUTTON_PIN_MODE(BUTTON_TIMER_MODE_PIN_ACTIVE_STATE));

    /* set baud rate in Config.hpp */
    serial_begin();
    Serial.println(F("Starting"));


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
        Serial.println(F("Multiple buttons pressed"));
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* calibrate internal adc reference if requested */
    if (internal_reference_set_button_pressed)
    {
        show_setup_result_final_block(setup_calibrate_internal_reference());
    }

    /* check avcc voltage, power down if too low for modules to work */
    float supply_voltage = get_supply_voltage();
    Serial.print(F("AVCC voltage: "));
    Serial.print(supply_voltage, 3);
    Serial.print(F("V ; Reference voltage: "));
    uint16_t reference_voltage;
    DataEEPROM::getInternalAdcReference(reference_voltage);
    Serial.print(reference_voltage);
    Serial.print(F(" V, Minimum required: "));
    Serial.print(AVCC_MIN_VOLTAGE, 3);
    Serial.println(F(" V"));
    if (supply_voltage < AVCC_MIN_VOLTAGE)
    {
        /* power down with led blinking */
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* self test hx711 */
    Serial.println(F("HX711 initializing"));

    /* power on scale and init library */
    ScaleModule::begin();
    bool scale_status = true;

    /* indicate start of procedure */
    builtin_led_on();

    /* wait for scale to be ready */
    scale_status &= ScaleModule::waitReadyTimeout();
    
    /* perform scale stabilization only if scale is ready */
    if (scale_status)
    {
        Serial.println(F("HX711 stabilizing"));

        /* stabilize scale using dummy readings */
        scale_status &= ScaleModule::stabilize();
    }
    
    builtin_led_off();

    /* if scale is stabilized, test */
    if (scale_status)
    {
        setup_scale_measure();

        /* calibrate or tare scale if requested */
        if (calibrate_button_pressed || tare_button_pressed)
        {
            Serial.println(F("Calibrate or tare operation requested, HX711 taring"));

            /* tare operation is required anyways for calibration */
            scale_status &= ScaleModule::tare();

            /* if failed to tare scale, block */
            if (!scale_status)
            {
                show_setup_result_final_block(RESULT_FAILURE);
            }

            /* get new tare offset */
            long new_tare_offset = ScaleModule::getOffset();

            /* store to EEPROM */
            DataEEPROM::setScaleTareOffset(new_tare_offset);
            
            Serial.print(F("Saved new tare offset: "));
            Serial.println(new_tare_offset);

            /* tare only operation requested */
            if (tare_button_pressed)
            {
                Serial.println(F("Only tare operation requested"));

                /* operation successful */
                show_setup_result_final_block(RESULT_SUCCESS);
            }

            /* calibrate scale */
            if (calibrate_button_pressed)
            {
                Serial.println(F("Calibrating scale"));

                setup_calibrate_scale_factor();

                /* block end of calibration */
                show_setup_result_final_block(RESULT_SUCCESS);
            }

            /* operation unknown */
            Serial.println(F("Unknown scale operation"));
            show_setup_result_final_block(RESULT_FAILURE);
        }
    }
    else
    {
        Serial.println(F("HX711 failed to stabilize"));
    }

    /* test DHT */
    Serial.println(F("DHT initializing"));

    /* initialize DHT sensor */
    DHTModule::begin();

    /* indicate start of procedure */
    builtin_led_on();

    /* wait for DHT sensor to power up */
    sleep_until(SLEEP_MODE_IDLE, DHT22PowerManager::poweredOn());

    builtin_led_off();

    /* check if DHT sensor is ready */
    bool dht_status = DHTModule::ready();

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

    /* test RTC */
    Serial.println(F("RTC initializing"));

    /* indicate start of procedure */
    builtin_led_on();

    /* power on rtc */
    RTCModule::preBeginPowerOn();

    /* wait for rtc to power up then start i2c communication */
    bool rtc_status = RTCModule::begin();

    /* assume power loss */
    bool power_loss = true;

    /* assume timer time has not been adjusted */
    bool timer_mode_time_adjusted = false;

    /* assume time is not valid */
    bool time_valid = false;

    DateTime dt = timer_mode_datetime;

    builtin_led_off();

    /* if rtc is ready */
    if (rtc_status)
    {
        /* check for power loss */
        power_loss = RTCModule::lostPower();

        if (power_loss)
        {
            Serial.println(F("RTC lost power, time is not valid."));

            if (timer_mode_button_pressed_on_boot)
            {
                Serial.println(F("Timer mode enabled. Setting time to 01-Jan-2025 00:00:00"));
                
                timer_mode_time_adjusted = RTCModule::adjust(timer_mode_datetime);

                if (!timer_mode_time_adjusted)
                {
                    Serial.println(F("Failed to set timer mode time"));
                }
            }
            else
            {
                Serial.println(F("Set the clock or use timer mode"));
            }
        }

        /* read time from rtc and print it */
        Serial.println(F("Reading time from RTC"));
        DateTime dt = RTCModule::now();

        /* check if rtc time is valid */
        time_valid &= dt.isValid();
        /* check if rtc time is greater than timer mode time */
        time_valid &= dt >= timer_mode_datetime;

        if (time_valid)
        {
            /* output date and time to buffer */
            char buffer[21] = "DD-MMM-YYYY hh:mm:ss";
            dt.toString(buffer);

            Serial.print(F("RTC time: "));
            Serial.println(buffer);
        }
        else
        {
            Serial.println(F("RTC time is invalid"));
        }
    }
    else
    {
        Serial.println(F("RTC failed to power up"));
    }

    /* test GSM, if test passes, send report */

    if (time_valid)
    {
        show_setup_result_final_block(RESULT_SUCCESS);
    }

    // dht_status - true if DHT passed the test
    // scale_status - true if scale passed the test
    // rtc_status - true if rtc is initialized
    // power_loss - true if rtc lost power
    // timer_mode_time_adjusted - true if timer mode time is set
    // time_valid - true if time is valid

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

void show_setup_result_final_block(bool success)
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
            Serial.println(F("Enter AREF voltage in milivolts: "));
            startTime = millis();

            builtin_led_on();
            sleep_idle_timeout_millis(100);
            builtin_led_off();
        }
    } while (!inputComplete);

    /* convert input to long */
    long inputLong = input.toInt();

    Serial.print(F("User input: "));
    Serial.print(inputLong);
    Serial.println(F(" mV"));

    /* set internal reference if within range */
    if ((inputLong > 1000 || inputLong < 1200))
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

        return true;
    }

    /* input is invalid */
    return false;
}

void setup_calibrate_scale_factor()
{

    float known_mass_kg = SCALE_CALIBRATION_KNOWN_MASS_KG;

    /* wait for user to place known weight and press calibrate button */
    do
    {
        Serial.print(F("Press tare button after "));
        Serial.print(known_mass_kg, 0);
        Serial.println(F(" kg weight is placed"));

        builtin_led_on();
        sleep_power_down_185ms_adc_off_bod_on();
        builtin_led_off();
        sleep_power_down_1065ms_adc_off_bod_on();
    } while (!BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE));

    /* set scale factor to 1 (default) */
    ScaleModule::setScale(1.0f);

    /* measure average 10x and calculate new scale factor */
    float new_scale_factor = ScaleModule::getUnits(10) / known_mass_kg;

    /* save new scale factor to EEPROM */
    DataEEPROM::setScaleCalibrationValue(new_scale_factor);

    Serial.print(F("Saved scale factor: "));
    Serial.println(new_scale_factor);
}

void setup_scale_measure()
{
    /* load scale parameters from EEPROM */
    long tare_offset;
    DataEEPROM::getScaleTareOffset(tare_offset);
    float scale_factor;
    DataEEPROM::getScaleCalibrationValue(scale_factor);

    /* Apply scale parameters */
    ScaleModule::setOffset(tare_offset);
    ScaleModule::setScale(scale_factor);

    Serial.print(F("Tare offset: "));
    Serial.println(tare_offset);
    Serial.print(F("Scale factor: "));
    Serial.println(scale_factor, 5);

    /* measure average 10x */
    float measured_mass_kg = ScaleModule::getUnits(10);

    Serial.print(F("Measured mass: "));
    Serial.print(measured_mass_kg, 3);
    Serial.println(F(" kg"));
}