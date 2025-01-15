#pragma once

#include <Arduino.h>
#include <DHT.h>
#include <power/DHT22PowerManager.hpp>

/**
 * @brief A class to manage operations for a single DHT sensor module.
 * 
 * This class provides static methods to initialize, read data, and manage 
 * the power state of a DHT sensor. It supports reading temperature and 
 * humidity, with optional pull-up timing and power management features.
 */
class DHTModule
{
public:
    /**
     * @brief Initializes the DHT sensor with optional pull-up time.
     * 
     * This method powers off the sensor momentarily before powering it back on 
     * and initializing it with the specified pull-up time.
     * 
     * @param pullup_time_us The pull-up time in microseconds (default is 55 µs).
     */
    static void begin(uint8_t pullup_time_us = 55);
    
    /**
     * @brief Powers off the DHT sensor and sets the data pin to input mode.
     */
    static void end();
    
    /**
     * @brief Checks if the sensor is ready to read data.
     * 
     * This method verifies that the sensor is powered on and ready to provide 
     * valid data.
     * 
     * @param[in] current_millis The current time in milliseconds. If not provided, 
     * `millis()` will be used.
     * 
     * @return `true` if the sensor is ready to read, otherwise `false`.
     */
    static bool ready(unsigned long current_millis = millis());
    
    /**
     * @brief Reads data from the DHT sensor.
     * 
     * If the `force` parameter is set to `true`, the sensor will be forced to 
     * read new data, bypassing any internal caching mechanism.
     * 
     * @param force Whether to force a new reading (default is `false`).
     * @return `true` if the reading was successful, otherwise `false`.
     */
    static bool read(bool force = false);
    
    /**
     * @brief Returns the temperature reading from the DHT sensor.
     * 
     * This method ensures that the sensor is powered on before attempting to 
     * read the temperature.
     * 
     * @return The temperature in Celsius, or `NAN` if the sensor is not powered on.
     */
    static float readTemperature();
    
    /**
     * @brief Returns the humidity reading from the DHT sensor.
     * 
     * This method ensures that the sensor is powered on before attempting to 
     * read the humidity.
     * 
     * @return The humidity percentage, or `NAN` if the sensor is not powered on.
     */
    static float readHumidity();
    
private:
    static DHT dht_; /**< Static instance of the DHT sensor class for managing sensor operations. */
};

