#include <Arduino.h>
#include <Config.hpp>
#include <power/SIM800PowerManager.hpp>
#include <module/GSMModule.hpp>
#include <SoftwareSerial.h>
#include <ResponseBuffer.hpp>
#include <serial.hpp>

#define STRINGIFY(x) #x
#define CONCAT_BAUD_RATE_SET_COMMAND(rate) "+IPR=" STRINGIFY(rate)

//static const char EMPTY_STR_P[] PROGMEM = "";
//#define EMPTY_FPSTR FPSTR(EMPTY_STR_P)

/* SIM800 responses */
static const char OK_STRING_P[] PROGMEM = "OK";

static const char RESPONSE_FMT_PARSER_CREG_CSQ_P[] PROGMEM = "%*[^+]+%*[^:]: %hhu,%hhu";


/**
 * @brief A dummy stream class for use when a stream is not available.
 * @details
 * This class implements the Stream interface and always returns
 * -1 for read and peek. Available returns 0. It also does nothing for flush.
 * write is also a no-op.
 */
class DummyStream : public Stream {
public:
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}
    size_t write(uint8_t) override { return 0; }
    using Print::write;
};

/**
 * @brief Clears the RX buffer of the given stream and sends an AT command with an optional
 *        command string.
 *
 * This function clears the RX buffer of the given stream using `stream_clear_rx_buffer()`
 * and then sends an AT command with an optional command string. The command string is
 * read from program memory using `strnlen_P()` and sent only if it is not empty and not
 * longer than `GSM_COMMAND_MAX_LEN`.
 *
 * @param stream Pointer to the Stream object whose RX buffer is to be cleared and to send the
 *               AT command.
 * @param command Pointer to the command string to send after the AT command. If `nullptr`, only
 *                the AT command is sent.
 * @return The number of bytes sent, including the AT command and the optional command string.
 */
size_t clear_rx_buffer_and_send_at_command(Stream* stream, const __FlashStringHelper* command)
{
    /* clear rx buffer and prepare it for new response */
    if (!stream_clear_rx_buffer(stream))
    {
        return 0;
    }

    /* count of sent bytes */
    size_t sent = 0;
    /* send AT */
    sent += stream->printf(F("AT"));

    /* if command is not null send it */
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
            sent += stream->printf(F("%S"), FP(command));
        }
    }
    
    /* send "\r\n" */
    sent += stream->printf(F("\r\n"));

    /* return number of bytes sent */
    return sent;
}

/**
 * @brief Sends an AT command and waits for a response.
 * 
 * This function is a convenience wrapper around `clear_rx_buffer_and_send_at_command` and `stream_wait_for_response`.
 * It sends the specified AT command and waits for a response within the specified timeout period.
 * If the response is available within the timeout period, the number of bytes read into the response buffer is returned.
 * If the timeout period expires, 0 is returned.
 * 
 * @param stream The stream to use for communication with the module.
 * @param command The AT command to send. If nullptr, only "AT" is sent.
 * @param response_timeout_ms The timeout in milliseconds to wait for a response. Defaults to `SIM800_SERIAL_TIMEOUT_MS`.
 * @return The number of bytes read into the response buffer, or 0 if the timeout period expires.
 */
size_t send_at_command_and_wait_for_response(Stream* stream, const __FlashStringHelper* command, unsigned long response_timeout_ms = SIM800_SERIAL_TIMEOUT_MS)
{
    if (!clear_rx_buffer_and_send_at_command(stream, command))
    {
        return 0;
    }

    return stream_wait_for_response(stream, response_timeout_ms);
}

/**
 * @brief Sends an AT command and waits for an "OK" response.
 * 
 * This function is a convenience wrapper around `send_at_command_and_wait_for_response`.
 * It sends the specified AT command and waits for a response within the specified timeout period.
 * If the response is available within the timeout period and contains the string "OK", true is returned.
 * If the timeout period expires, or the response does not contain "OK", false is returned.
 * 
 * @param stream The stream to use for communication with the module.
 * @param command The AT command to send. If nullptr, only "AT" is sent.
 * @param response_timeout_ms The timeout in milliseconds to wait for a response. Defaults to `SIM800_SERIAL_TIMEOUT_MS`.
 * @return true if "OK" is received, false otherwise.
 */
bool send_at_command_and_expect_ok(Stream* stream, const __FlashStringHelper* command = nullptr, unsigned long response_timeout_ms = SIM800_SERIAL_TIMEOUT_MS)
{
    return send_at_command_and_wait_for_response(stream, command, response_timeout_ms) &&
           (strstr_P(response_buffer, OK_STRING_P) != nullptr);
}

/* Static variable to track the readiness of the GSM Module */
bool GSMModule::ready_ = false;
bool GSMModule::sms_receive_enabled_ = false;

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
    GSMModule::sms_receive_enabled_ = false;

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
                bool echo_off = send_at_command_and_expect_ok(GSMModule::software_serial_, F("E0"));
                if (echo_off && baud_saved) /* if baud rate was saved break the loop */
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

/*
* @brief Check if the SIM800 GSM module is registered on a network.
* 
* This method uses the `AT+CREG?` command to determine the registration status of the module.
* If the module is registered, the method returns `true`, otherwise it returns `false`.
* 
* @return `true` if the module is registered on a network, otherwise `false`.
*/
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
    if (!GSMModule::ready_ || !send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CREG?"), 5000UL))
    {
        return false;
    }
    
    /* variables to store parsed data */
    uint8_t creg_n;
    uint8_t creg_stat = 0;
    serial_printf(F("CREG resp:\n%s\n"), response_buffer);
    /* parse state, return false if parsing failed or its not 1 or 5 */
    return (sscanf_P(response_buffer, RESPONSE_FMT_PARSER_CREG_CSQ_P, &creg_n, &creg_stat) == 2) 
            && ((creg_stat == 1) || (creg_stat == 5));
}

/**
 * @brief Retrieves the current signal quality of the GSM module.
 * 
 * This method uses the `AT+CSQ` command to determine the signal quality of the module.
 * If the module is not ready or the command fails, the method returns `false`.
 * Otherwise, it returns `true` and the retrieved values are stored in the `csq_rssi` and `csq_ber` parameters.
 * 
 * @param[out] csq_rssi The received signal strength indication, in the range of 0 to 31 or 99.
 * @param[out] csq_ber The channel bit error rate, in the range of 0 to 7 or 99.
 * @return `true` if the command was successful and values are in range and not 99, otherwise `false`.
 */
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

    serial_printf(F("CSQ resp:\n%s\n"), response_buffer);
    /* parse state, return false if parsing failed or parameters are out of range for valid signal */
    return (sscanf_P(response_buffer, RESPONSE_FMT_PARSER_CREG_CSQ_P, &csq_rssi, &csq_ber) == 2) 
            && ((csq_rssi <= 31) || (csq_ber <= 7));
}

/**
 * @brief Sends a text message using the SIM800 GSM module.
 * 
 * This method uses the `AT+CMGF=1` and `AT+CMGS` commands to send a text message.
 * If the module is not ready or the command fails, the method returns `false`.
 * Otherwise, it returns `true` if the message is sent successfully.
 * 
 * @param number The phone number of the recipient of the message, up to 15 characters.
 * @param message The message to be sent, up to 159 characters.
 * @return `true` if the message is sent successfully, otherwise `false`.
 */
bool GSMModule::sendSMS(const char* number, const char* message)
{
    if (!GSMModule::ready_ || number == nullptr || message == nullptr || !send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CMGF=1")))
    {
        return false;
    }

    /* get length of number */
    size_t number_len = strnlen(number, 16);
    /* get length of message */
    size_t message_len = strnlen(message, 160);
    serial_printf(F("number_len: %u, message_len: %u\n"), number_len, message_len);
    serial_flush();
    /* make sure that parameters are null terminated and valid */
    if (number_len < 10 || number_len == 16 || message_len == 160)
    {
        return false;
    }

    serial_printf(F("SMS->+%s:\n%s\n"), number, message);

    /* clear rx buffer before sending the command */
    stream_clear_rx_buffer(GSMModule::software_serial_);

    /* send the command */
    GSMModule::software_serial_->printf(F("AT+CMGS=\"+%s\"\r\n"), number);

    /* wait for response to prompting to enter sms content */
    if (GSMModule::software_serial_->readBytesUntil('>', response_buffer, sizeof(response_buffer)) == 0)
    {
        response_buffer[sizeof(response_buffer) - 1] = 0;
        serial_printf(F("No response: %s"), response_buffer);
        return false;
    }
    
    /* prepare rx buffer for the CMGS response */
    stream_clear_rx_buffer(GSMModule::software_serial_);

    /* send message content and end it with ctrl+z (char 26) */
    GSMModule::software_serial_->printf(F("%s"), message);
    GSMModule::software_serial_->write((char)26);

    /* wait for response up to 60 seconds */
    stream_wait_for_response(GSMModule::software_serial_, 60000UL);

    /* TP-Message-Reference (TP-MR) field in the GSM 03.40 */
    uint8_t tp_mr = 0;
    serial_printf(F("response_buffer: %s"), response_buffer);
    /* check if OK is received */
    return sscanf_P(response_buffer, PSTR("%*[^+]+CMGS: %hhu%*[^O]OK"), tp_mr) == 1;
}

bool GSMModule::enableSMSReceive()
{
    serial_printf(F("SMS RX EN"));
    if (!GSMModule::ready_)
    {
        return false;
    }

    GSMModule::sms_receive_enabled_ = send_at_command_and_expect_ok(GSMModule::software_serial_, F("+CNMI=2,2,0,0,0"));

    /* if result is true, clean rx buffer, be ready for reception */
    if (GSMModule::sms_receive_enabled_)
    {
        stream_clear_rx_buffer(GSMModule::software_serial_);
    }

    return GSMModule::sms_receive_enabled_;
}

bool GSMModule::receiveSMS(char* sms_content_buffer, size_t sms_content_buffer_size, unsigned long timeout_ms, char* sms_sender_number_buffer, size_t sms_sender_number_buffer_size)
{
    /* setup sms response mode */
    if (!GSMModule::ready_ || sms_content_buffer == nullptr)
    {
        return false;
    }

    unsigned long startTime = millis();
    const char* rx_cmt = nullptr;
    do
    {
        stream_wait_for_response(GSMModule::software_serial_, 0);
        rx_cmt = strstr_P(response_buffer, PSTR("+CMT: \"+"));
    } while ((rx_cmt != nullptr) && ((millis() - startTime) < timeout_ms));
    
    serial_printf(F("RX:\n%s\n:RX\n"), response_buffer);

    /* if rx_cmt is still nullptr then timeout occured */
    if (rx_cmt == nullptr)
    {
        return false;
    }

    /* both buffers are supplied but both are 0 in len or just content buffer is supplied and its 0 in len, just return true */
    if ((sms_sender_number_buffer != nullptr && sms_sender_number_buffer_size == 0 && sms_content_buffer_size == 0) ||
        (sms_content_buffer_size == 0))
    {
        return true;
    }

    // parse the number
    const char* numberStart = rx_cmt + 8; // "+CMT: "+" - 8 chars
    const char* numberEnd = strstr_P(numberStart, PSTR("\","));
    /* if senders number couldnt be parsed, +CMT URC is invalid */
    if (numberEnd == nullptr)
    {
        return false;
    }

    /* if phone number does not end with digit then +CMT response is invalid */
    if (!isdigit(*(numberEnd-1)))
    {
        return false;
    }

    /* example response: +CMT: "+<number>","<contant name>","<time>"\r\n<sms_content> */
    /* find pointer to start of message content */
    const char* startOfContent = strstr_P(rx_cmt, PSTR("\"\r\n"));
    if (startOfContent == nullptr)
    {
        return false;
    }

    /* make startOfContent actually point to start of content */
    startOfContent += 3; // add "<cr><lf>

    /* return false if sms_content_buffer_size is not big enough for content */
    if (strnlen(startOfContent, sms_content_buffer_size) >= sms_content_buffer_size)
    {
        return false; // Buffer too small, content is truncated
    }

    /* copy the content to buffer */
    strlcpy(sms_content_buffer, startOfContent, sms_content_buffer_size);

    /* null terminate the content buffer just in case */
    sms_content_buffer[sms_content_buffer_size - 1] = '\0';

    /* if sender number buffer is not provided or len is 0, return true since were done with content copy */
    if (sms_sender_number_buffer == nullptr || sms_sender_number_buffer_size == 0)
    {
        return true;
    }

    /* null terminate the senders phone number */    
    *const_cast<char*>(numberEnd) = '\0';

    strlcpy(sms_sender_number_buffer, numberStart, sms_sender_number_buffer_size);

    sms_sender_number_buffer[sms_sender_number_buffer_size - 1] = '\0';

    return true;
}

/**
 * @brief Powers off the SIM800 GSM module and resets the serial pins.
 *
 * This function should be called when the GSM module is no longer needed
 * to save power.
 */
void GSMModule::end()
{
    GSMModule::ready_ = false;
    GSMModule::sms_receive_enabled_ = false;

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

/**
 * @brief Returns a reference to the stream object.
 *
 * This function is used to get a reference to the stream object. If the GSM module is not ready
 * or the `software_serial_` object is null, it returns a reference to a static DummyStream object.
 * Otherwise, it returns a reference to the `software_serial_` object.
 *
 * @return A reference to the stream object.
 */
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