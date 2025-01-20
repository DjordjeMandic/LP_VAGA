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

//static const char EMPTY_STR_P[] PROGMEM = "";
//#define EMPTY_FPSTR FPSTR(EMPTY_STR_P)

/* SIM800 responses */
static const char OK_STRING_P[] PROGMEM = "OK";
static const char ERROR_STRING_P[] PROGMEM = "ERROR";

static const char RESPONSE_FMT_PARSER_CREG_CSQ_P[] PROGMEM = "%*[^+]+%*[^:]: %hhu,%hhu";

static_assert(GSM_RESPONSE_BUFFER_SIZE >= _SS_MAX_RX_BUFF, "GSM_RESPONSE_BUFFER_SIZE must be at least _SS_MAX_RX_BUFF.");
/* SIM800 response buffer */
static char response_buffer[GSM_RESPONSE_BUFFER_SIZE];

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

    /* clear serial rx buffer */
    while (stream->available() > 0)
    {
        stream->read();
    }

    /* clear response_buffer */
    memset(response_buffer, '\0', sizeof(response_buffer));

    return true;
}

size_t clear_rx_buffer_and_send_at_command(Stream* stream, const __FlashStringHelper* command)
{
    if (!stream_clear_rx_buffer(stream))
    {
        return 0;
    }

    /* count of sent bytes */
    size_t sent = 0;
    /* send AT */
    sent += stream->print(F("AT"));

    if (command != nullptr)
    {
        /* sim800 rx buffer is 556bytes */
        static_assert(GSM_COMMAND_MAX_LEN < 556 && GSM_COMMAND_MAX_LEN >= 0, "GSM_COMMAND_MAX_LEN must be greater than 0 and less than 556.");

        /* get lenght of command */
        size_t cmd_len = strnlen_P(FP(command), GSM_COMMAND_MAX_LEN);

        /* cmd_len equals to GSM_COMMAND_MAX_LEN if no '\0' was found */
        if (cmd_len > 0 && cmd_len < GSM_COMMAND_MAX_LEN)
        {
            /* send command only if it is not empty and not too long */
            sent += stream->print(command);
        }
    }
    
    /* send "\r\n" */
    sent += stream->println();

    /* return number of bytes sent */
    return sent;
}

size_t send_at_command_and_copy_response_to(Stream* stream, const __FlashStringHelper* command, char* buffer, size_t buffer_size, unsigned long response_timeout_ms = SIM800_SERIAL_TIMEOUT_MS)
{
    if (buffer == nullptr || buffer_size == 0 || !clear_rx_buffer_and_send_at_command(stream, command))
    {
        return 0;
    }

    /* wait for response*/
    size_t chars_read_into_buffer = 0;
    unsigned long start = millis();
    do
    {
        chars_read_into_buffer = stream->readBytes(buffer, buffer_size);
    } while ((chars_read_into_buffer == 0) && (millis() - start < response_timeout_ms));
    
    /* null terminate buffer */
    buffer[chars_read_into_buffer] = '\0';

    return chars_read_into_buffer;
}

bool send_at_command_and_expect_ok(Stream* stream, const __FlashStringHelper* command = nullptr, unsigned long response_timeout_ms = SIM800_SERIAL_TIMEOUT_MS)
{
    return send_at_command_and_copy_response_to(stream, command, response_buffer, sizeof(response_buffer), response_timeout_ms) &&
           (strstr_P(response_buffer, OK_STRING_P) != nullptr);
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

bool GSMModule::begin(unsigned long current_millis, bool save_baud_rate)
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

    static_assert(SIM800_SERIAL_TIMEOUT_MS > 200, "SIM800_SERIAL_TIMEOUT_MS must be greater than 200.");

    /* start listening */
    GSMModule::software_serial_->begin(SIM800_BAUD_RATE);
    GSMModule::software_serial_->setTimeout(SIM800_SERIAL_TIMEOUT_MS);

    static_assert(SIM800_AUTOBAUD_ATTEMPTS > 0, "SIM800_AUTOBAUD_ATTEMPTS must be greater than 0.");

    /* assume baud rate is saved, will get set to false if saving baud rate is requested and it fails */
    bool baud_saved = true;

    /* start autobaud */
    for (int i = 0; i < SIM800_AUTOBAUD_ATTEMPTS; i++)
    {
        /* send at, wait for ok response */
        if (send_at_command_and_expect_ok(GSMModule::software_serial_))
        {
            /* send command to set baud rate and wait for ok response */
            if (send_at_command_and_expect_ok(GSMModule::software_serial_, F(CONCAT_BAUD_RATE_SET_COMMAND(SIM800_BAUD_RATE))))
            {
                /* if saving baud rate is enabled */
                if (save_baud_rate)
                {
                    /* save baud to memory and store result */
                    baud_saved = send_at_command_and_expect_ok(GSMModule::software_serial_, F("&W"));
                }
                
                if (baud_saved) /* if baud rate was saved break the loop */
                {
                    GSMModule::ready_ = true;
                    break;
                }
            }
        }

        /* sleep for 100ms if response is not ok */
        sleep_idle_timeout_millis(100);
    }

    return GSMModule::ready_;
}

bool GSMModule::registeredOnNetwork()
{
    /*
    * Execution Command Response:
    *   \r\n+CREG: <n>,<stat>\r\nOK\r\n
    *   If an error occurs related to ME functionality:
    *     +CME ERROR: <err>
    *
    * Parameters:
    *   <n> (Network Registration Unsolicited Result Code Setting):
    *     - 0  Disable network registration unsolicited result code
    *     - 1  Enable network registration unsolicited result code
    *          (+CREG: <stat>)
    *     - 2  Enable network registration unsolicited result code with location information
    *          (+CREG: <stat>[,<lac>,<ci>])
    *
    *   <stat> (Network Registration Status):
    *     - 0  Not registered, MT is not currently searching for a new operator
    *     - 1  Registered, home network
    *     - 2  Not registered, but MT is currently searching for a new operator
    *     - 3  Registration denied
    *     - 4  Unknown
    *     - 5  Registered, roaming
    */

    /* if module is not ready or command failed, return false */
    if (!GSMModule::ready_ || !send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CREG?"), 10000UL))
    {
        return false;
    }
    
    /* variables to store parsed data */
    uint8_t creg_n = 0;
    uint8_t creg_stat = 0;

    /* parse state, return false if parsing failed or its not 1 or 5 */
    return (sscanf_P(response_buffer, RESPONSE_FMT_PARSER_CREG_CSQ_P, &creg_n, &creg_stat) == 2) 
            && ((creg_stat == 1) || (creg_stat == 5));
}

bool GSMModule::signalQuality(uint8_t& csq_rssi, uint8_t& csq_ber)
{
    /*
    * Execution Command Response:
    *   \r\n+CSQ: <rssi>,<ber>\r\nOK\r\n
    *   If an error occurs related to ME functionality:
    *     +CME ERROR: <err>
    *
    * Parameters:
    *   <rssi> (Received Signal Strength Indication):
    *     - 0        : -115 dBm or less
    *     - 1        : -111 dBm
    *     - 2...30   : -110 to -54 dBm (linear mapping)
    *     - 31       : -52 dBm or greater
    *     - 99       : Not known or not detectable
    *
    *   <ber> (Channel Bit Error Rate, in percent):
    *     - 0...7    : RXQUAL values as defined in GSM 05.08, subclause 7.2.4
    *     - 99       : Not known or not detectable
    */

    /* if module is not ready or command failed, return false */
    if (!GSMModule::ready_ || !send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CSQ")))
    {
        return false;
    }

    /* parse state, return false if parsing failed or its not 1 or 5 */
    return (sscanf_P(response_buffer, RESPONSE_FMT_PARSER_CREG_CSQ_P, &csq_rssi, &csq_ber) == 2) 
            && ((csq_rssi <= 31) || (csq_ber <= 7));
}

bool GSMModule::sendSMS(const char* number, const char* message)
{
    if (!GSMModule::ready_ || number == nullptr || message == nullptr || !send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CMGF=1")))
    {
        return false;
    }

    static_assert(GSM_NUMBER_TEXT_MAX_LEN <= 15 && GSM_NUMBER_TEXT_MAX_LEN >= 10, "GSM_NUMBER_TEXT_MAX_LEN = 15 digits, minimum 10");
    size_t number_len = strnlen(number, 16);

    static_assert(GSM_SMS_TEXT_MAX_LEN > 0 && GSM_SMS_TEXT_MAX_LEN <= 159, "GSM_SMS_TEXT_MAX_LEN = 159 characters for single sms");
    size_t message_len = strnlen(message, 160);

    if (number_len < 10 || number_len == 16 || message_len == 160)
    {
        return false;
    }

    /* clear rx buffer before sending the command */
    stream_clear_rx_buffer(GSMModule::software_serial_);

    /* send the command */
    GSMModule::software_serial_->printf(F("AT+CMGS=\"+%s\"\r\n"), number);

    /* wait for response */
    if (GSMModule::software_serial_->readBytesUntil('>', response_buffer, sizeof(response_buffer)) == 0)
    {
        return false;
    }
    
    /* prepare rx buffer for the CMGS response */
    stream_clear_rx_buffer(GSMModule::software_serial_);

    /* send message content and end it with ctrl+z (char 26) */
    GSMModule::software_serial_->printf(F("%s%c"), message, 26);

    /* set response timeout to 60s */
    unsigned long oldStreamTimeout = GSMModule::software_serial_->getTimeout();
    GSMModule::software_serial_->setTimeout(60000UL);

    /* wait for response */
    GSMModule::software_serial_->readBytes(response_buffer, sizeof(response_buffer));

    /* restore old timeout */
    GSMModule::software_serial_->setTimeout(oldStreamTimeout);
// fix this because timeout will wait 60s anyawys. as soon as +CMGS: <code> OK is found, return
    /* check if OK is received */
    return strstr_P(response_buffer, OK_STRING_P) != nullptr;
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