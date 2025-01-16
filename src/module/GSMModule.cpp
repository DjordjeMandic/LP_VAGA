#include <Arduino.h>
#include <Config.hpp>
#include <power/SIM800PowerManager.hpp>
#include <module/GSMModule.hpp>
#include <SoftwareSerial.h>

#define STRINGIFY(x) #x
#define CONCAT_BAUD_RATE_SET_COMMAND(rate) "+IPR=" STRINGIFY(rate)

/* casts to const __FlashStringHelper pointer */
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))

/* casts to PGM_P */
#define FP(fsh_pointer) (reinterpret_cast<PGM_P>(fsh_pointer))

static const char EMPTY_STR_P[] PROGMEM = "";
#define EMPTY_FPSTR FPSTR(EMPTY_STR_P)

/* SIM800 responses */
static const char OK_STRING_P[] PROGMEM = "OK";
static const char ERROR_STRING_P[] PROGMEM = "ERROR";

static_assert(GSM_FORMAT_BUFFER_SIZE >= _SS_MAX_RX_BUFF, "GSM_FORMAT_BUFFER_SIZE must be at least _SS_MAX_RX_BUFF.");
/* SIM800 response buffer */
static char response_buffer[GSM_FORMAT_BUFFER_SIZE];

class DummyStream : public Stream {
public:
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}
    size_t write(uint8_t) override { return 0; }
    using Print::write;
};

bool stream_clear_rx_buffer(Stream* stream)
{
    if (stream == nullptr)
    {
        return false;
    }

    while (stream->available() > 0)
    {
        stream->read();
    }

    return true;
}

size_t clear_rx_buffer_and_send_at_command(Stream* stream, const __FlashStringHelper* command)
{
    if (command == nullptr || !stream_clear_rx_buffer(stream))
    {
        return 0;
    }

    /* count of sent bytes */
    size_t sent = 0;
    /* send AT */
    sent += stream->print(F("AT"));

    static_assert(GSM_COMMAND_MAX_LEN >= _SS_MAX_RX_BUFF, "GSM_COMMAND_MAX_LEN must be greater than or equal to _SS_MAX_RX_BUFF.");

    /* get lenght of command */
    size_t cmd_len = strnlen_P(FP(command), GSM_COMMAND_MAX_LEN);

    if (cmd_len > 0 && cmd_len < GSM_COMMAND_MAX_LEN)
    {
        /* send command only if it is not empty and not too long */
        sent += stream->print(command);
    }

    /* send "\r\n" */
    sent += stream->println();

    /* return number of bytes sent */
    return sent;
}

size_t send_at_command_and_copy_response_to(Stream* stream, const __FlashStringHelper* command, char* buffer, size_t buffer_size)
{
    if (buffer == nullptr || buffer_size == 0 || !clear_rx_buffer_and_send_at_command(stream, command))
    {
        return 0;
    }

    /* wait for response*/
    static_assert(SIM800_RESPONSE_TIMEOUT_MS > 200, "SIM800_RESPONSE_TIMEOUT_MS must be greater than 200.");
    unsigned long start = millis();
    size_t index = 0;
    while (millis() - start < SIM800_RESPONSE_TIMEOUT_MS)
    {
        /* break if buffer is full */
        if (index == buffer_size - 1)
        {
            break;
        }

        /* if data available */
        if (stream->available() > 0)
        {
            /* read data from stream */
            buffer[index++] = stream->read();

            /* restart timeout */
            start = millis();
        }
        else
        {
            /* no data available */
            sleep_idle_timeout_millis(10);
        }
    }

    /* null terminate buffer */
    buffer[index] = '\0';

    return index;
}

bool send_at_command_and_expect(Stream* stream, const __FlashStringHelper* command, const __FlashStringHelper* expected_in_response)
{
    if (expected_in_response == nullptr || !send_at_command_and_copy_response_to(stream, command, response_buffer, sizeof(response_buffer)))
    {
        return false;
    }

    return strstr_P(response_buffer, FP(expected_in_response)) != nullptr;
}

bool send_at_command_and_expect_ok(Stream* stream, const __FlashStringHelper* command)
{
    return send_at_command_and_expect(stream, command, FPSTR(OK_STRING_P));
}

/* Static variable to track the readiness of the GSM Module */
bool GSMModule::ready_ = false;

/* Static variable to store pointer to the SoftwareSerial instance */
SoftwareSerial* GSMModule::software_serial_ = nullptr;

void GSMModule::preBeginPowerOn()
{
    /* Power off the SIM800 to reset it */
    SIM800PowerManager::powerOff();
    sleep_idle_timeout_millis(1);

    /* Power on the SIM800 */
    SIM800PowerManager::powerOn();
}

bool GSMModule::begin(unsigned long current_millis)
{
    GSMModule::ready_ = false;
    
    /* Power on the SIM800 if needed */
    if (!SIM800PowerManager::powerEnabled())
    {
        SIM800PowerManager::powerOn();
    }

    /* make sure its powered on */
    sleep_idle_timeout_millis(SIM800PowerManager::requiredPowerOnDelay(current_millis));
    
    /* initialize the SoftwareSerial instance */
    if (GSMModule::software_serial_ == nullptr)
    {
        static SoftwareSerial sim_module(SIM800_RX_PIN, SIM800_TX_PIN);
        GSMModule::software_serial_ = &sim_module;
    }
    else
    {
        /* stop listening while pins are being reconfigured */
        GSMModule::software_serial_->end();
    }

    /* set TX pin to output with HIGH idle state */
    digitalWrite(SIM800_TX_PIN, HIGH);
    pinMode(SIM800_TX_PIN, OUTPUT);

    /* set RX pin to input with pullup */
    pinMode(SIM800_RX_PIN, INPUT_PULLUP);

    static_assert(
        (SIM800_BAUD_RATE == 1200) ||
        (SIM800_BAUD_RATE == 2400) ||
        (SIM800_BAUD_RATE == 4800) ||
        (SIM800_BAUD_RATE == 9600) ||
        (SIM800_BAUD_RATE == 19200) ||
        (SIM800_BAUD_RATE == 38400),
        "SIM800_BAUD_RATE must be one of: 1200, 2400, 4800, 9600, 19200, 38400");

    static_assert(SIM800_RESPONSE_TIMEOUT_MS > 200, "SIM800_RESPONSE_TIMEOUT_MS must be greater than 200.");

    /* start listening */
    GSMModule::software_serial_->begin(SIM800_BAUD_RATE);
    GSMModule::software_serial_->setTimeout(SIM800_RESPONSE_TIMEOUT_MS);

    static_assert(SIM800_AUTOBAUD_ATTEMPTS > 0, "SIM800_AUTOBAUD_ATTEMPTS must be greater than 0.");

    /* start autobaud */
    for (int i = 0; i < SIM800_AUTOBAUD_ATTEMPTS; i++)
    {
        if (send_at_command_and_expect_ok(GSMModule::software_serial_, EMPTY_FPSTR))
        {
            if (send_at_command_and_expect_ok(GSMModule::software_serial_, F(CONCAT_BAUD_RATE_SET_COMMAND(SIM800_BAUD_RATE))))
            {
                GSMModule::ready_ = true;
                break;
            }
        }

        /* sleep for 100ms */
        sleep_idle_timeout_millis(100);
    }

    return GSMModule::ready_;
}

bool GSMModule::registeredOnNetwork()
{
    /* if module is not ready or command failed, return false */
    if (!GSMModule::ready_ || !send_at_command_and_copy_response_to(GSMModule::software_serial_, F("+CREG?"), response_buffer, sizeof(response_buffer)))
    {
        return false;
    }

    /* look for +CREG: x,y in the response */
    static const char key_fp[] PROGMEM = "+CREG:";
    const char* start = strstr_P(response_buffer, key_fp);

    if (start != nullptr)
    {
        /* move past key */
        start += strlen_P(key_fp);

        /* parse the first number (x) and skip to the second */
        char* end;
        strtol(start, &end, 10);

        if (*end == ',')
        {
            /* parse the second number (y) */
            long reg_status = strtol(end + 1, nullptr, 10);

            /* check if registered */
            return reg_status == 1 || reg_status == 5; // 1: Home, 5: Roaming
        }
    }

    /* failed to parse, not registered */
    return false;
}

void GSMModule::end()
{
    GSMModule::ready_ = false;

    /* stop listening */
    if (GSMModule::software_serial_ != nullptr)
    {
        /* there is no TX buffering, simply stop listening */
        GSMModule::software_serial_->end();
    }

    /* reset pins */
    pinMode(SIM800_RX_PIN, INPUT);
    pinMode(SIM800_TX_PIN, INPUT);

    /* power off the SIM800 */
    SIM800PowerManager::powerOff();
}

Stream& GSMModule::getStream()
{
    /* If the module is not ready, return a dummy stream */
    if (GSMModule::software_serial_ == nullptr || !GSMModule::ready_)
    { 
        static DummyStream dummyStream; // Static dummy instance
        return dummyStream;
    }

    return *GSMModule::software_serial_;
}