#include <Arduino.h>
#include <Config.hpp>
#include <power/SIM800PowerManager.hpp>
#include <module/GSMModule.hpp>
#include <SoftwareSerial.h>

#define STRINGIFY(x) #x
#define CONCAT_AT_BAUD_RATE_COMMAND(rate) "AT+IPR=" STRINGIFY(rate)

/* SIM800 OK response */
static const char* SIM800_OK_RESPONSE = "OK";


static_assert(GSM_FORMAT_BUFFER_SIZE >= _SS_MAX_RX_BUFF, "GSM_FORMAT_BUFFER_SIZE must be at least _SS_MAX_RX_BUFF.");
/* SIM800 response buffer used for scanf */
static char formatted_io_buffer[GSM_FORMAT_BUFFER_SIZE];

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

size_t send_command(Stream* stream, const __FlashStringHelper* command)
{
    if (!stream_clear_rx_buffer(stream))
    {
        return false;
    }
    stream->println(command);              // Send command and "\r\n"
}

bool send_command_and_expect(Stream* stream, const __FlashStringHelper* command, const char* expected_response = SIM800_OK_RESPONSE)
{
    if (!send_command(stream, command))
    {
        return false;
    }
    return stream->find((char*)expected_response); // Wait for expected response
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

    static_assert(SIM800_RESPONSE_TIMEOUT_MS > 100, "SIM800_RESPONSE_TIMEOUT_MS must be greater than 100.");

    /* start listening */
    GSMModule::software_serial_->begin(SIM800_BAUD_RATE);
    GSMModule::software_serial_->setTimeout(SIM800_RESPONSE_TIMEOUT_MS);

    bool baud_set = false;

    static_assert(SIM800_AUTOBAUD_ATTEMPTS > 0, "SIM800_AUTOBAUD_ATTEMPTS must be greater than 0.");

    /* start autobaud */
    for (int i = 0; i < SIM800_AUTOBAUD_ATTEMPTS; i++)
    {
        if (send_command_and_expect(GSMModule::software_serial_, F("AT"), SIM800_OK_RESPONSE))
        {
            if (send_command_and_expect(GSMModule::software_serial_, F(CONCAT_AT_BAUD_RATE_COMMAND(SIM800_BAUD_RATE)), SIM800_OK_RESPONSE))
            {
                baud_set = true;
                break;
            }
        }

        /* sleep for 100ms */
        sleep_idle_timeout_millis(100);
    }

    /* return false if baud rate could not be set */
    if (!baud_set)
    {
        return false;
    }

    return baud_set;
}

bool GSMModule::registeredOnNetwork()
{
    /* if module is not ready or command failed, return false */
    if (!GSMModule::ready_ || !send_command(GSMModule::software_serial_, F("AT+CREG?")))
    {
        return false;
    }



    /* clear the formatted io buffer */
    memset(formatted_io_buffer, 0, sizeof(formatted_io_buffer));

}

bool GSMModule::end()
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