#pragma once

#include <Arduino.h>
#include <Config.hpp>   


static_assert(_SS_MAX_RX_BUFF >= 256, "SS_MAX_RX_BUFF must be at least 256.");

#define RESPONSE_BUFFER_SIZE _SS_MAX_RX_BUFF

static_assert(RESPONSE_BUFFER_SIZE >= _SS_MAX_RX_BUFF, "RESPONSE_BUFFER_SIZE must be at least _SS_MAX_RX_BUFF.");
extern char response_buffer[RESPONSE_BUFFER_SIZE];

/**
 * @brief Clears the RX buffer of the given stream.
 *
 * This function clears both the internal response buffer and the RX buffer of the specified stream.
 * It reads from the stream until no more data is available, effectively discarding any incoming data.
 *
 * @param stream Pointer to the Stream object whose RX buffer is to be cleared.
 * @return Returns true if the stream is valid and the buffer is cleared successfully, false otherwise.
 */
bool stream_clear_rx_buffer(Stream* stream);

/**
 * @brief Reads from the specified Stream until a response is available or a timeout occurs.
 *
 * This function reads from the specified stream until a response is available or a timeout occurs.
 * If the stream is null, the function returns immediately with a chars_read_into_buffer of 0.
 * If the stream is valid, the function reads until a response is available or the timeout specified
 * by `response_timeout_ms` is reached. The response is stored in the `response_buffer` global variable.
 * The number of characters read into the buffer is returned.
 *
 * @param stream Pointer to the Stream object to read from.
 * @param response_timeout_ms The timeout in milliseconds to wait for a response.
 * @return The number of characters read into the `response_buffer` global variable.
 */
size_t stream_wait_for_response(Stream* stream, unsigned long response_timeout_ms = SIM800_SERIAL_TIMEOUT_MS);