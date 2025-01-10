#pragma once

/**
 * @brief Enables power to the TWI (I2C) peripheral.
 *        This function uses the `power_twi_enable()` macro from `<avr/power.h>`
 *        to turn on the TWI module, allowing I2C communication to be used.
 */
void twi_power_on();

/**
 * @brief Disables power to the TWI (I2C) peripheral.
 *        This function stops the I2C communication using `Wire.end()`, 
 *        disables the TWI module with `power_twi_disable()`, and sets
 *        the SDA and SCL pins to `INPUT` mode to reduce power consumption.
 */
void twi_power_off();
