#pragma once

#include <Arduino.h>
#include <HX711.h>
#include <power/HX711PowerManager.hpp>

#define SCALE_STABILIZATION_TIME_MS 3000

/**
 * @brief A class to manage operations for a single HX711 scale module.
 * 
 * This class provides static methods to initialize, read data, and manage 
 * the power state of the HX711 scale module. It includes methods for 
 * stabilization, taring, and retrieving scaled values.
 */
class ScaleModule
{
public:
    /**
     * @brief Initializes the scale module.
     * 
     * This method powers off the scale momentarily before powering it back on 
     * and initializing it with the defined data and clock pins.
     */
    static void begin();

    /**
     * @brief Powers off the scale module and sets its data pins to input mode.
     */
    static void end();

    /**
     * @brief Checks if the scale module is ready for reading.
     * 
     * @param[in] current_millis The current time in milliseconds. If not provided, 
     * `millis()` will be used.
     * 
     * @return `true` if the scale is ready, otherwise `false`.
     */
    static bool ready(unsigned long current_millis = millis());

    /**
     * @brief Waits for the scale module to be ready.
     */
    static bool waitReady();

    /**
     * @brief Waits for the scale module to be ready, retrying a specified number of times.
     * 
     * @param[in] retries Number of retries before giving up.
     * @param[in] delay_ms Delay (in milliseconds) between retries.
     * @return `true` if the scale becomes ready, otherwise `false`.
     */
    static bool waitReadyRetry(const int retries = 3, const unsigned long delay_ms = 100);

    /**
     * @brief Waits for the scale module to be ready, with a timeout.
     * 
     * @param[in] timeout Maximum time (in milliseconds) to wait.
     * @param[in] delay_ms Delay (in milliseconds) between checks.
     * @return `true` if the scale becomes ready, otherwise `false`.
     */
    static bool waitReadyTimeout(const unsigned long timeout = 1000 + HX711_POWER_DELAY_MS, const unsigned long delay_ms = 100);

    /**
     * @brief Stabilizes the scale by reading it for a specified time.
     * 
     * @param[in] stabilization_time_ms Time (in milliseconds) to stabilize.
     * @return `true` if stabilization was successful, otherwise `false`.
     */
    static bool stabilize(const uint16_t stabilization_time_ms = SCALE_STABILIZATION_TIME_MS);

    /**
     * @brief Reads raw data from the scale module.
     * 
     * @return The raw reading from the scale.
     */
    static long read();

    /**
     * @brief Reads the average of multiple raw data samples.
     * 
     * @param[in] times Number of samples to average.
     * @return The averaged raw reading.
     */
    static long readAverage(const uint8_t times = 10);

    /**
     * @brief Returns the raw value relative to the offset.
     * 
     * @param[in] times Number of samples to average.
     * @return The raw value relative to the offset.
     */
    static double getValue(const uint8_t times = 1);

    /**
     * @brief Returns the scaled units.
     * 
     * @param[in] times Number of samples to average.
     * @return The scaled units.
     */
    static float getUnits(const uint8_t times = 1);

    /**
     * @brief Tares the scale by setting the current reading as zero.
     * 
     * @param[in] times Number of samples to average for taring.
     * @return `true` if taring was successful, otherwise `false`.
     */
    static bool tare(const uint8_t times = 10);

    /**
     * @brief Sets the scale factor.
     * 
     * @param[in] scale The scale factor to set.
     */
    static void setScale(const float scale = 1.f);

    /**
     * @brief Returns the current scale factor.
     * 
     * @return The current scale factor.
     */
    static float getScale();

    /**
     * @brief Sets the offset for the scale.
     * 
     * @param[in] offset The offset to set.
     */
    static void setOffset(const long offset = 0);

    /**
     * @brief Returns the current offset.
     * 
     * @return The current offset.
     */
    static long getOffset();

private:
    static HX711 scale_; /**< Static instance of the HX711 class for managing scale operations. */
    static bool ready_; /**< Stores the readiness state of the scale. */
};
