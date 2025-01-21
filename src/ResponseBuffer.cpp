#include <Arduino.h>
#include <Config.hpp>
#include <ResponseBuffer.hpp>

char response_buffer[RESPONSE_BUFFER_SIZE];

bool stream_clear_rx_buffer(Stream* stream)
{
    if (stream == nullptr)
    {
        return false;
    }

    /* clear response_buffer */
    memset(response_buffer, '\0', sizeof(response_buffer));

    /* clear serial rx buffer */
    while (stream->available() > 0)
    {
        stream->read();
    }

    return true;
}

size_t stream_wait_for_response(Stream* stream, unsigned long response_timeout_ms)
{
    /* wait for response*/
    size_t chars_read_into_buffer = 0;

    /* if stream is not null read from it */
    if (stream != nullptr)
    {
        unsigned long start = millis();
        do
        {
            chars_read_into_buffer = stream->readBytes(response_buffer, sizeof(response_buffer));
        } while ((chars_read_into_buffer == 0) && (millis() - start < response_timeout_ms));
    }

    /* null terminate buffer since readBytes does not */
    response_buffer[chars_read_into_buffer >= sizeof(response_buffer) ? sizeof(response_buffer) - 1 : chars_read_into_buffer] = '\0';

    return chars_read_into_buffer;
}
