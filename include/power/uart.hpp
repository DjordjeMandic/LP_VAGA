#pragma once

/**
 * @brief Enables power to the UART (USART0) peripheral.
 *        This function uses the `power_usart0_enable()` macro from `<avr/power.h>`
 *        to turn on the UART module, allowing serial communication to be used.
 */
void uart_power_on();

/**
 * @brief Disables power to the UART (USART0) peripheral.
 *        This function uses the `power_usart0_disable()` macro from `<avr/power.h>`
 *        to turn off the UART module to save power. Additionally, it sets the UART
 *        TX (pin 1) and RX (pin 0) pins to `INPUT` mode to further reduce power consumption.
 */
void uart_power_off();