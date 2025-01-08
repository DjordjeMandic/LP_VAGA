#include <Arduino.h>
#include <LowPower.h>
#include <HX711.h>
#include <power/hx711.h>
#include <power/ds3231.h>
#include <power/gsm.h>
#include <module/scale.h>




void setup()
{
    gsm_power_off();
    hx711_power_off();
    ds3231_power_off();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    scale_begin();
    Serial.begin(9600);
    Serial.println(F("Start"));
    while(!hx711_power_delay_check())
    {
        Serial.println(F("Waiting for HX711 to power on..."));
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