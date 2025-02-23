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
#include <ResponseBuffer.hpp>

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

const DateTime timer_mode_datetime = DateTime(2025, 1, 1);

bool setup_calibrate_internal_reference();
bool setup_calibrate_scale_factor();

uint8_t sms_hour_of_day = DataEEPROM::getSMSReportHourOfTheDay();
uint16_t internal_reference = DataEEPROM::getInternalAdcReference();
float scale_factor = DataEEPROM::getScaleCalibrationValue();
long scale_tare_offset = DataEEPROM::getScaleTareOffset();
float calculate_supply_voltage(uint16_t adc_value, uint16_t reference_voltage = internal_reference);

#define NEW_LINE "\n"

static const char GSM_REQUIRED_STRING_P[] PROGMEM = STRING_GSM_REQUIRED NEW_LINE;
static const char STARTUP_MESSAGE_P[] PROGMEM = STRING_STARTUP_MESSAGE NEW_LINE;

char smsBuffer[GSM_SMS_TEXT_MAX_LEN + 1];

static const char* const POWER_ON_SELF_TEST_RESULT_STRING[] = {"FAIL", "PASS"};

typedef union {
    uint8_t u8_value; // The entire 8-bit value
    struct {
        uint8_t scale_status : 1;
        uint8_t dht_status : 1;
        uint8_t rtc_status : 1;
        uint8_t rtc_lost_power : 1;
        uint8_t rtc_time_valid : 1;
        uint8_t rtc_timer_mode : 1;
        uint8_t gsm_status : 1;
        uint8_t post_pass : 1;
    } fields; // Individual bits as named fields
} power_on_self_test_result_t;

power_on_self_test_result_t power_on_self_test_result = { .u8_value = 0 };

char phone_number_to_send_sms[16];

bool send_sms(const char* number, const char* message);
bool check_sms_buffer_ovf(int snprintf_result);
uint8_t mapCSQToSignalLevel(const uint8_t csq);
bool set_internal_reference(uint16_t input_mv);
DateTime date_time;

void rtc_alarm_isr()
{

}

void setup()
{
    power_all_off();
    
    /* alive signal */
    builtin_led_on();

    /* configure buttons */
    pinMode(BUTTON_CALIBRATE_PIN, BUTTON_PIN_MODE(BUTTON_CALIBRATE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TARE_PIN, BUTTON_PIN_MODE(BUTTON_TARE_PIN_ACTIVE_STATE));
    pinMode(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_PIN_MODE(BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE));
    pinMode(BUTTON_TIMER_MODE_PIN, BUTTON_PIN_MODE(BUTTON_TIMER_MODE_PIN_ACTIVE_STATE));

    /* set baud rate in Config.hpp */
    serial_begin();
    Serial.printf(FPSTR(STARTUP_MESSAGE_P));

    if (sms_hour_of_day > 23) {
        sms_hour_of_day = 21;
    }

    /* read buttons */
    bool calibrate_button_pressed = BUTTON_PRESSED(BUTTON_CALIBRATE_PIN, BUTTON_CALIBRATE_PIN_ACTIVE_STATE);
    bool tare_button_pressed      = BUTTON_PRESSED(BUTTON_TARE_PIN, BUTTON_TARE_PIN_ACTIVE_STATE);
    bool internal_reference_set_button_pressed = BUTTON_PRESSED(BUTTON_INTERNAL_REFERENCE_SET_PIN, BUTTON_INTERNAL_REFERENCE_SET_PIN_ACTIVE_STATE);
    timer_mode_button_pressed_on_boot = BUTTON_PRESSED(BUTTON_TIMER_MODE_PIN, BUTTON_TIMER_MODE_PIN_ACTIVE_STATE);
    serial_printf(F("A0:%u\nA1:%u\nA2:%u\nA3:%u\n" NEW_LINE), calibrate_button_pressed, tare_button_pressed, timer_mode_button_pressed_on_boot, internal_reference_set_button_pressed);
    /* count number of pressed buttons */
    uint8_t button_pressed_count = calibrate_button_pressed + tare_button_pressed + internal_reference_set_button_pressed + timer_mode_button_pressed_on_boot;
    /* enable sms config mode only if calibrate and tare are pressed */
    bool sms_config_mode = calibrate_button_pressed && tare_button_pressed && (button_pressed_count == 2);
    /* if multiple buttons are pressed, block */
    if (button_pressed_count > 1)
    {
        /* if not in sms config mode */
        if (!sms_config_mode)
        {
            Serial.printf(F(STRING_MULTIPLE_BUTTONS_PRESSED NEW_LINE));
            show_setup_result_final_block(RESULT_FAILURE);
        }
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
        Serial.printf(F(STRING_REF_INVALID " %u" NEW_LINE), internal_reference);
        set_internal_reference(internal_reference);
        show_setup_result_final_block(RESULT_FAILURE);
    }
    /* check min avcc voltage constant */
    static_assert(float(AVCC_MIN_VOLTAGE) >= 3.5f, "AVCC_MIN_VOLTAGE must be above 3.5V for modules to work");
    static_assert(float(AVCC_MIN_VOLTAGE) < 4.2f, "AVCC_MIN_VOLTAGE must be below 4.2V");
    float supply_voltage = get_supply_voltage(); /* read supply voltage */
    Serial.printf(F("AVCC:%.3fV MIN:%.3fV REF:%umV\n"), supply_voltage, float(AVCC_MIN_VOLTAGE), internal_reference);
    /* check avcc voltage, power down if too low for modules to work */
    if (supply_voltage < AVCC_MIN_VOLTAGE)
    {
        /* power down with led blinking */
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* test hx711 */
    Serial.printf(F(STRING_SCALE_INITIALIZING NEW_LINE));
    ScaleModule::begin(); /* power on scale and init library */
    /* apply scale parameters */
    ScaleModule::setOffset(scale_tare_offset);
    ScaleModule::setScale(scale_factor);
    Serial.printf(F(STRING_TARE_OFFSET_SC_SPACE "%ld" NEW_LINE STRING_SCALE_FACTOR_SC_SPACE "%.5f" NEW_LINE), scale_tare_offset, scale_factor);
    bool scale_status = true;
    builtin_led_on(); /* indicate start of procedure */
    scale_status &= ScaleModule::waitReadyTimeout(); /* wait for scale to be ready */
    /* perform scale stabilization only if scale is ready */
    if (scale_status)
    {
        Serial.printf(F(STRING_SCALE_STABILIZING NEW_LINE));
        scale_status &= ScaleModule::stabilize(); /* stabilize scale using dummy readings */
    }
    builtin_led_off();
    show_setup_result_serial_only(scale_status);
    power_on_self_test_result.fields.scale_status = scale_status;
    float measured_10_times_avg = NAN;
    /* if scale is stabilized, test */
    if (scale_status)
    {
        /* Measure average 10x */
        measured_10_times_avg = ScaleModule::getUnits(10);
        Serial.printf(F(STRING_MEASURED_MASS_SC_SPACE "%.3f" STRING_SPACE_KG NEW_LINE), measured_10_times_avg);

        /* calibrate or tare scale if requested */
        if (!sms_config_mode && (calibrate_button_pressed || tare_button_pressed))
        {
            Serial.printf(F(STRING_SCALE_TARING NEW_LINE));
            scale_status &= ScaleModule::tare(); /* tare operation is required anyways for calibration */
            /* if failed to tare scale, block */
            if (!scale_status)
            {
                show_setup_result_final_block(RESULT_FAILURE);
            }
            scale_tare_offset = ScaleModule::getOffset(); /* get new tare offset */
            DataEEPROM::setScaleTareOffset(scale_tare_offset); /* store to EEPROM */
            Serial.printf(F(STRING_SAVED_TARE_OFFSET_SC_SPACE "%ld" NEW_LINE), scale_tare_offset);

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
            Serial.printf(F(STRING_UNKNOWN_COMMAND NEW_LINE));
            show_setup_result_final_block(RESULT_FAILURE);
        }
    }

    /* test DHT */
    Serial.printf(F(STRING_DHT_INITIALIZING NEW_LINE));
    DHTModule::begin(); /* initialize DHT sensor */
    builtin_led_on(); /* indicate start of procedure */
    sleep_until(SLEEP_MODE_IDLE, DHT22PowerManager::poweredOn()); /* wait for DHT sensor to power up */
    builtin_led_off();
    bool dht_status = DHTModule::ready(); /* check if DHT sensor is ready */
    float temp = DHTModule::readTemperature();
    float humidity = DHTModule::readHumidity();
    Serial.printf(F("Temp: %.1fC ; Humidity: %.1f" NEW_LINE), temp, humidity);
    /* check if temperature and humidity are finite */
    dht_status &= isfinite(temp);
    dht_status &= isfinite(humidity);
    show_setup_result_serial_only(dht_status);
    power_on_self_test_result.fields.dht_status = dht_status;

    /* test RTC */
    Serial.printf(F(STRING_RTC_INITIALIZING NEW_LINE));
    RTCModule::preBeginPowerOn(); /* power on rtc */
    builtin_led_on(); /* indicate start of procedure */
    bool rtc_status = false; /* assume rtc is not ready */
    bool rtc_lost_power = true; /* assume power loss */
    bool rtc_time_valid = false; /* assume time is not valid */
    bool rtc_timer_mode_time_adjusted = false; /* assume timer time has not been adjusted */


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
            Serial.printf(F(STRING_RTC_LOST_POWER NEW_LINE));
            if (!timer_mode_button_pressed_on_boot)
            {
                Serial.printf(F(STRING_SET_CLOCK_OR_USE_TIMER_MODE NEW_LINE));
                break; /* time is not valid and timer mode is not enabled */
            }
            Serial.printf(F(STRING_TIMER_ENABLED_SETTING_TIME NEW_LINE));
            rtc_timer_mode_time_adjusted = RTCModule::adjust(timer_mode_datetime);
            if (!rtc_timer_mode_time_adjusted)
            {
                break;
            }
        }

        Serial.printf(F(STRING_READ_RTC_TIME NEW_LINE));
        date_time = RTCModule::now(); /* read time from rtc */
        rtc_time_valid = date_time.isValid(); /* check if rtc time is valid */
        if (!rtc_time_valid)
        {
            Serial.printf(F(STRING_RTC_TIME_INVALID NEW_LINE));
            break;
        }

        /* if rtc time is valid print it */
        Serial.printf(
            F(STRING_RTC_TIME_SC_SPACE "%02u/%02u/%04u %02u:%02u:%02u\n"),
            date_time.day(),     // DD
            date_time.month(),   // MM
            date_time.year(),    // YYYY
            date_time.hour(),    // HH
            date_time.minute(),  // MM
            date_time.second()   // SS
        );

        /* check if rtc time is greater than or equal to timer mode time */
        rtc_time_valid &= date_time >= timer_mode_datetime;
    } while (false);
    show_setup_result_serial_only(rtc_time_valid);
    power_on_self_test_result.fields.rtc_status = rtc_status;
    power_on_self_test_result.fields.rtc_lost_power = rtc_lost_power;
    power_on_self_test_result.fields.rtc_time_valid = rtc_time_valid;
    power_on_self_test_result.fields.rtc_timer_mode = rtc_timer_mode_time_adjusted && timer_mode_button_pressed_on_boot;

    /* test GSM, if test passes, send report */

    Serial.printf(F(STRING_GSM_INITIALIZING NEW_LINE));
    GSMModule::preBeginPowerOn(); /* power on SIM800 */
    bool gsm_status = GSMModule::begin(); /* initialize GSM module */
    show_setup_result_serial_only(gsm_status);

    do
    {
        /* check if module is ready */
        if (!gsm_status)
        {
            break;
        }

        /* wait for module to register on network */
        Serial.printf(F(STRING_GSM_REGISTERING_ON_NETWORK NEW_LINE));
        unsigned long gsm_start_millis = millis();
        bool registered_on_network = false;
        do
        {
            /* check if module is registered on network */
            registered_on_network = GSMModule::registeredOnNetwork();
            /* if not registered on network, sleep for 2 seconds */
            if (!registered_on_network)
            {
                Serial.printf(F("Waiting for network registration" NEW_LINE));
                sleep_idle_timeout_millis(2000);
            }
            /* wait for 60 seconds or until registered on network */
        } while (!registered_on_network && millis() - gsm_start_millis < 120000UL);
        show_setup_result_serial_only(registered_on_network);
        gsm_status &= registered_on_network;
    } while (false);
    power_on_self_test_result.fields.gsm_status = gsm_status;
    power_on_self_test_result.fields.post_pass = power_on_self_test_result.fields.dht_status &&
                                                 power_on_self_test_result.fields.scale_status &&
                                                 power_on_self_test_result.fields.rtc_status &&
                                                 power_on_self_test_result.fields.rtc_time_valid &&
                                                 power_on_self_test_result.fields.gsm_status;

    Serial.printf(F("Power on self test: %s\n"), POWER_ON_SELF_TEST_RESULT_STRING[static_cast<bool>(power_on_self_test_result.fields.post_pass)]);

    date_time = RTCModule::now();

    int result = snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("%S\n1:%.3f\n2:%.3f\n3:%u\n4:%ld\n5:%.2f\n6:%.1f\n7:%.1f\n8:%.1f\n9:%u\n%02u/%02u/%04u-%02u:%02u:%02u\n\n0x%02X %s"), 
                                        STARTUP_MESSAGE_P, 
                                        supply_voltage, // 1
                                        float(AVCC_MIN_VOLTAGE), // 2
                                        internal_reference, // 3
                                        scale_tare_offset, // 4
                                        scale_factor, // 5
                                        measured_10_times_avg, // 6
                                        temp, // 7
                                        humidity, // 8
                                        sms_hour_of_day, // 9
                                        date_time.day(), date_time.month(), date_time.year(), date_time.hour(), date_time.minute(), date_time.second(),
                                        power_on_self_test_result.u8_value,
                                        POWER_ON_SELF_TEST_RESULT_STRING[static_cast<bool>(power_on_self_test_result.fields.post_pass)]);
    
    
    Serial.printf(F("SMS:\n%s\n"), smsBuffer);

    check_sms_buffer_ovf(result);

    if (!gsm_status)
    {
        Serial.printf(FPSTR(GSM_REQUIRED_STRING_P));
        show_setup_result_final_block(RESULT_FAILURE);
    }

    if (!DataEEPROM::getPhoneNumber(phone_number_to_send_sms, sizeof(phone_number_to_send_sms))) {
        Serial.printf(F("Phone number not set\n"));
        show_setup_result_final_block(RESULT_FAILURE);
    }

    bool send_status = GSMModule::sendSMS(phone_number_to_send_sms, smsBuffer);
    show_setup_result_serial_only(send_status);

    /* sms sending failed, block */
    if (!send_status)
    {
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* if entered, must be reset manually by pressing reset button */
    if (sms_config_mode)
    {
        /* prepare config sms info */
        result = snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("Config:" NEW_LINE NEW_LINE
                                                                "AREF:1000...1200" NEW_LINE
                                                                "RTC:DD/MM/YYYY-HH:MM" NEW_LINE
                                                                "SMS:HH" NEW_LINE
                                                                "OK-OFF" NEW_LINE
                                                                NEW_LINE
                                                                "AVCC: %f V"), get_supply_voltage());
                                                                
        send_status = GSMModule::sendSMS(phone_number_to_send_sms, smsBuffer);
        show_setup_result_serial_only(send_status);

        if (!send_status || !GSMModule::enableSMSReceive())
        {
            show_setup_result_final_block(RESULT_FAILURE);
        }
        
        /* initialize adc for internal reference calibration */
        ADCHelper::refBGInit();

        bool shouldSend = false;
        char sender[16];
        unsigned long start = millis();
        do
        {
            if (shouldSend) {
                send_status &= GSMModule::sendSMS(phone_number_to_send_sms, smsBuffer);
                shouldSend = false;
            }
            memset(sender, '\0', sizeof(sender));
            memset(smsBuffer, '\0', sizeof(smsBuffer));
            sleep_idle_timeout_millis(1000);
            /* read incoming sms content and store it in smsBuffer */
            /* switch trough sms commands AREF, RTC, SMS */
            /* parse each command data */
            if (!GSMModule::receiveSMS(smsBuffer, sizeof(smsBuffer), 0, sender, sizeof(sender))) {
                continue;
            }
            serial_printf(F("SMS RX +%s:\n%s\nEND\n"), sender, smsBuffer);

            uint16_t aref, year;
            uint8_t day, month, hour, minute, sms_hour;
            shouldSend = true;
            if (5 == sscanf_P(smsBuffer, PSTR("RTC:%" SCNu8 "/%" SCNu8 "/%" SCNu16 "-%" SCNu8 ":%" SCNu8), &day, &month, &year, &hour, &minute)) {
                bool adjust_result = power_on_self_test_result.fields.rtc_status && RTCModule::adjust(DateTime(year, month, day, hour, minute));
                snprintf_P(smsBuffer, sizeof(smsBuffer), adjust_result ? PSTR("OK") : PSTR("ERR"));
                continue;
            }

            if (1 == sscanf_P(smsBuffer, PSTR("AREF:%" SCNu16), &aref)) {
                bool result = set_internal_reference(aref);
                snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("AREF: %u mV\nAVCC: %f V\n%S"), internal_reference, get_supply_voltage(), result ? PSTR("OK") : PSTR("ERR"));
                continue;
            }

            if (1 == sscanf_P(smsBuffer, PSTR("SMS:%" SCNu8), &sms_hour)) {
                if (sms_hour < 24) {
                    sms_hour_of_day = sms_hour;
                    DataEEPROM::setSMSReportHourOfTheDay(sms_hour_of_day);
                }
                snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("SMS: %u\n%S"), sms_hour_of_day, sms_hour_of_day == sms_hour ? PSTR("OK") : PSTR("ERR"));
                continue;
            }

            shouldSend = false;
            if (strstr_P(smsBuffer, PSTR("OK-OFF")) != nullptr) {
                show_setup_result_final_block(RESULT_SUCCESS);
            }
        } while (send_status && (millis() - start < 600000UL)); /* 10 minute timeout */
        show_setup_result_final_block(RESULT_FAILURE);
    }

    // dht_status - true if DHT passed the test
    // scale_status - true if scale passed the test
    // rtc_status - true if rtc is initialized
    // rtc_lost_power - true if rtc lost power
    // timer_mode_time_adjusted - true if timer mode time is set
    // time_valid - true if time is valid


    /* test failed, display result and block, do not continue to loop */
    if (!power_on_self_test_result.fields.post_pass) {
        show_setup_result_final_block(RESULT_FAILURE);
    }

    power_all_off();
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
}

/* loop is only for low power optimized operation */
void loop()
{
    unsigned long elapsedSinceStart = 0;
    /* woken up, detach interrupt and turn off alarm, set new one */
    serial_begin();
    ScaleModule::begin(); /* 500ms needed */
    RTCModule::preBeginPowerOn(); /* 500ms needed */
    GSMModule::preBeginPowerOn(); /* 5000ms needed */

    sleep_power_down_565ms_adc_off_bod_on();
    elapsedSinceStart += 565UL;
    serial_printf(F("RTC on\n"));
    if (!RTCModule::begin(millis() + elapsedSinceStart)) {
        show_setup_result_final_block(RESULT_FAILURE);
    }

    /* read date and time */
    DateTime alarm_time = RTCModule::now();
    if (!alarm_time.isValid()) {
        show_setup_result_final_block(RESULT_FAILURE);
    }
    RTCModule::end();
    
    /* check battery voltage under load */
    float battery_voltage = get_supply_voltage();
    serial_printf(F("AVCC: %f V\n"), battery_voltage);
    if (battery_voltage < AVCC_MIN_VOLTAGE) {
        show_setup_result_final_block(RESULT_FAILURE);
    }

    date_time = alarm_time;
    if (power_on_self_test_result.fields.rtc_timer_mode) {
        alarm_time = date_time + TimeSpan(1, 0, 0, 0);
    } else {
        alarm_time = DateTime(date_time.year(), date_time.month(), date_time.day(), sms_hour_of_day, 0, 0);
        if (date_time.hour() >= sms_hour_of_day) {
            alarm_time = alarm_time + TimeSpan(1, 0, 0, 0);
        }
    }

    DHTModule::begin(); /* 2050ms needed */

    /* stabilize the scale */
    if (ScaleModule::waitReadyRetry(10, 100UL)) {
        ScaleModule::setScale(scale_factor);
        ScaleModule::setOffset(scale_tare_offset);
        ScaleModule::stabilize(3000U);
        elapsedSinceStart += 3000;
    } else {
        sleep_idle_timeout_millis(2050U);
    }

    float temp = NAN;
    float humi = NAN;
    if (DHTModule::ready(millis() + 2500U)) {
        temp = DHTModule::readTemperature();
        humi = DHTModule::readHumidity();
    }
    DHTModule::end();

    float weight = ScaleModule::getUnits(10);
    ScaleModule::end();
    float last_weight_24 = DataEEPROM::getLastMeasurementKg();
    DataEEPROM::setLastMeasurementKg(weight);
    float diff24 = weight - last_weight_24;

    bool error = true;
    do {
        error = !GSMModule::begin(millis() + elapsedSinceStart);

        if (error) {
            GSMModule::end();
        } else {
            bool registeredOnNetwork = false;
            unsigned long start = millis();
            do {
                registeredOnNetwork = GSMModule::registeredOnNetwork();
                if (!registeredOnNetwork) {
                    sleep_idle_timeout_millis(2000U);
                }
            } while (!registeredOnNetwork && (millis() - start < 120000UL)); /* 2 minute timeout */
    
            error = !registeredOnNetwork;
        }


        RTCModule::preBeginPowerOn();
        uint8_t rssi = 99;
        uint8_t signalLevel = 99;

        GSMModule::signalQuality(rssi, signalLevel);
        signalLevel = mapCSQToSignalLevel(rssi);
        battery_voltage = get_supply_voltage();
        RTCModule::begin();
        DateTime now = RTCModule::now();
        if (now.isValid()) {
            date_time = now;
        }
        bool alarmSet = RTCModule::setWakeupAlarm(alarm_time, Ds3231Alarm1Mode::DS3231_A1_Date);
        RTCModule::end();

        serial_printf(F("Alarm: %02u:%02u:%02u - %02u.%02u.%04u\n"), alarm_time.hour(), alarm_time.minute(), alarm_time.second(), alarm_time.day(), alarm_time.month(), alarm_time.year());
        if (!alarmSet) {
            show_setup_result_serial_only(RESULT_FAILURE);
            error = true;
        }
        
        memset(smsBuffer, '\0', sizeof(smsBuffer));
        snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("[Masa]: %.2fkg" NEW_LINE
                                                      "[Unos]: %.2fkg" NEW_LINE
                                                      "[Temp]: %.1fC" NEW_LINE
                                                      "[Vlaga]: %.1f%%" NEW_LINE
                                                      "[Signal]: %u/9" NEW_LINE
                                                      "[Napon]: %.3fV" NEW_LINE
                                                      "[Vreme]: %02u:%02u:%02u %02u.%02u.%04u." NEW_LINE
                                                      "[Alarm]: %02uh %02u.%02u.%04u.%S"),
                                                             weight,
                                                             diff24,
                                                             temp,
                                                             humi,
                                                             signalLevel,
                                                             battery_voltage,
                                                             date_time.hour(),
                                                             date_time.minute(),
                                                             date_time.second(),
                                                             date_time.day(),
                                                             date_time.month(),
                                                             date_time.year(),
                                                             alarm_time.hour(),
                                                             alarm_time.day(),
                                                             alarm_time.month(),
                                                             alarm_time.year(),
                                                             error ? PSTR("\nGRESKA!") : PSTR(""));
        serial_printf(F("\n%s\n"), smsBuffer);
        GSMModule::sendSMS(phone_number_to_send_sms, smsBuffer);
        GSMModule::end();
    } while (0);

    if (error) {
        show_setup_result_final_block(RESULT_FAILURE);
    }

    power_all_off();
    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtc_alarm_isr, FALLING);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(RTC_INT_PIN));
}

bool check_sms_buffer_ovf(int snprintf_result)
{
    if (snprintf_result >= static_cast<int>(sizeof(smsBuffer)) || snprintf_result < 0)
    {
        snprintf_P(smsBuffer, sizeof(smsBuffer), PSTR("%S\nERROR-OVF:%d\nPOST:0x%02X %s"), STARTUP_MESSAGE_P, power_on_self_test_result.u8_value, POWER_ON_SELF_TEST_RESULT_STRING[static_cast<bool>(power_on_self_test_result.fields.post_pass)]);
        serial_printf(F("SMS-OVF:\n%s\n"), smsBuffer);
        return true;
    }
    return false;
}

void power_all_off()
{
    /* todo replace this with GSMModule */
    GSMModule::end();

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

    
    pinMode(RTC_INT_PIN, INPUT);
    pinMode(ACCEL_INT_PIN, INPUT);
}

bool send_sms(const char* number, const char* message)
{
    serial_printf(F(STRING_SENDING_SMS NEW_LINE));
    return GSMModule::sendSMS(number, message);
}

void show_setup_result_serial_only(bool success)
{
    if (serial_is_enabled())
    {
        Serial.printf(F(STRING_OPERATION_RESULT_SC_SPACE "%S" NEW_LINE), success ? PSTR(STRING_OK) : PSTR(STRING_FAIL));
    }
}

void show_setup_result_final_block(bool success)
{
    show_setup_result_serial_only(success);
    serial_printf(F(STRING_HALTED NEW_LINE));
    detachInterrupt(digitalPinToInterrupt(RTC_INT_PIN));
    detachInterrupt(digitalPinToInterrupt(ACCEL_INT_PIN));

    power_all_off();


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

    Serial.printf(F(STRING_MEASURE_VOLTAGE_ON_AREF_PIN NEW_LINE));

    uint16_t input_mv = 0;
    /* wait for user to input the voltage */
    do
    {
        stream_clear_rx_buffer(&Serial);
        
        /* heartbeat led and serial prompt */
        Serial.printf(F(STRING_ENTER_AREF_IN_MV NEW_LINE));
        builtin_led_on();
        sleep_idle_timeout_millis(100);
        builtin_led_off();

        size_t rx_count = stream_wait_for_response(&Serial, 5000UL);

        /* digits + newline */
        if (rx_count > 1 && sscanf_P(response_buffer, PSTR("%hu\n"), &input_mv) != 1)
        {
            input_mv = 0;
        }

        Serial.printf(F("input_mv: %u, RX %zu bytes: %s" NEW_LINE), input_mv, rx_count, response_buffer);
    } while (input_mv > 1200 || input_mv < 1000);

    return set_internal_reference(input_mv);
}

bool set_internal_reference(uint16_t input_mv)
{
    bool in_range = true;
    Serial.printf(F(STRING_INPUT_SC_SAPCE "%u" NEW_LINE), input_mv);
    if (input_mv < 1000 || input_mv > 1200) {
        in_range = false;
        input_mv = 1100;
    }

    /* save to EEPROM */
    internal_reference = input_mv;
    DataEEPROM::setInternalAdcReference(internal_reference);

    Serial.printf(F(STRING_SAVED_SC_SAPCE "%u" NEW_LINE), internal_reference);

    /* redo measurement */
    float voltage = get_supply_voltage();
    Serial.printf(F(STRING_SUPPLY_VOLTAGE_SC_SPACE "%.3f" STRING_SPACE_V NEW_LINE), voltage);

    return (in_range && (internal_reference == DataEEPROM::getInternalAdcReference())) ? RESULT_SUCCESS : RESULT_FAILURE;
}

bool setup_calibrate_scale_factor()
{
    static_assert(float(SCALE_CALIBRATION_KNOWN_MASS_KG) != 0.0f, "Known mass must be non-zero.");

    float known_mass_kg = SCALE_CALIBRATION_KNOWN_MASS_KG;

    if (!isfinite(known_mass_kg))
    {
        Serial.printf(F(STRING_KNOWN_MASS_NOT_FINITE NEW_LINE));
        return RESULT_FAILURE;
    }

    /* wait for user to place known weight and press calibrate button */
    do
    {
        Serial.printf(F(STRING_PLACE_SPACE "%.1f" STRING_SPACE_KG_ON_SCALE_AND_PRESS_TARE NEW_LINE), known_mass_kg);

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
        Serial.printf(F(STRING_NEW_SCALE_FACTOR_INVALID NEW_LINE));
        return RESULT_FAILURE;
    }

    /* save new scale factor to EEPROM */
    DataEEPROM::setScaleCalibrationValue(scale_factor);

    Serial.printf(F(STRING_SAVED_FACTOR_SC_SPACE " %.5f" NEW_LINE), scale_factor);

    return scale_factor == DataEEPROM::getScaleCalibrationValue() ? RESULT_SUCCESS : RESULT_FAILURE;
}

uint8_t mapCSQToSignalLevel(const uint8_t csq) {
    if (csq < 1 || csq > 31) {
      return 0;         // No signal / Unknown
    }
    if (csq == 1) {
      return 1;         // -111 dBm
    }
    if (csq == 31) {
      return 9;         // Strongest signal
    }

    // Map CSQ 2-30 to range 2-8
    return static_cast<uint8_t>((((float)(csq - 2) * (8 - 2)) / (30 - 2)) + 2.5f);  // Rounding at 0.5
}