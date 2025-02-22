#include <Arduino.h>
#include <avr/eeprom.h>
#include <DataEEPROM.hpp>
#include <stdint.h>
#include <stddef.h>

constexpr uint8_t key = 0xAA;  // Arbitrary secret key

// Rotate left an 8-bit value.
constexpr uint8_t rotl(uint8_t value, unsigned int shift) {
    return (value << shift) | (value >> (8 - shift));
}

// Rotate right an 8-bit value.
constexpr uint8_t rotr(uint8_t value, unsigned int shift) {
    return (value >> shift) | (value << (8 - shift));
}

// Encrypt a single character using its index.
constexpr char encrypt_char(char c, size_t i) {
    uint8_t tmp = static_cast<uint8_t>(c) + static_cast<uint8_t>(i);
    tmp ^= (key + static_cast<uint8_t>(i));
    return static_cast<char>(rotl(tmp, i % 8));
}

// Decrypt a single character (inverse of encrypt_char).
constexpr char decrypt_char(char c, size_t i) {
    uint8_t tmp = rotr(static_cast<uint8_t>(c), i % 8);
    tmp ^= (key + static_cast<uint8_t>(i));
    return static_cast<char>(tmp - static_cast<uint8_t>(i));
}

// Define a fixed-size structure to hold the encoded string.
template<size_t N>
struct EncodedString {
    char data[N] = {};
};

// Maximum phone number length (including the null terminator).
constexpr size_t maxPhoneLen = 16;

// Compile-time encoding into a fixed-size EncodedString<maxPhoneLen>.
// The input is a null-terminated literal of length N (including the null terminator).
template<size_t N>
constexpr EncodedString<maxPhoneLen> encode_fixed(const char (&input)[N]) {
    static_assert(N <= maxPhoneLen, "Input too long.");
    EncodedString<maxPhoneLen> output = {};
    // Process each character from the input (except the null terminator).
    for (size_t i = 0; i < N - 1; ++i) {
        output.data[i] = encrypt_char(input[i], i);
    }
    // Place a null terminator immediately after the encoded characters.
    if (N - 1 < maxPhoneLen) {
        output.data[N - 1] = '\0';
    }
    // The remaining elements (if any) are left as zero.
    return output;
}


// Runtime decryption function that recovers the original phone number.
template<size_t N>
void decode(const EncodedString<N>& encoded, char (&output)[N]) {
    for (size_t i = 0; i < N; i++) {
        // Stop at the null terminator.
        if (encoded.data[i] == '\0') {
            output[i] = '\0';
            break;
        }
        output[i] = decrypt_char(encoded.data[i], i);
    }
}

// Encrypt a phone number at compile time.
constexpr auto encodedPhone = encode_fixed("385989986336");

/* Static variables in EEPROM */
float EEMEM DataEEPROM::lastMeasurementKgEemem_ = 0.0f;
float EEMEM DataEEPROM::scaleCalibrationValueEemem_ = 1.0f;
long EEMEM DataEEPROM::scaleTareOffsetEemem_ = 0;
uint16_t EEMEM DataEEPROM::internalAdcReferenceEemem_ = 1100U;
uint8_t EEMEM DataEEPROM::smsReportHourOfTheDayEemem_ = 21U;
char EEMEM DataEEPROM::phoneNumberEemem_[maxPhoneLen] = {
    encodedPhone.data[0], encodedPhone.data[1], encodedPhone.data[2], encodedPhone.data[3],
    encodedPhone.data[4], encodedPhone.data[5], encodedPhone.data[6], encodedPhone.data[7],
    encodedPhone.data[8], encodedPhone.data[9], encodedPhone.data[10],
    encodedPhone.data[11], encodedPhone.data[12], encodedPhone.data[13],
    encodedPhone.data[14], encodedPhone.data[15]
};

/* Getters */

bool DataEEPROM::getPhoneNumber(char* phone_number_buffer, uint8_t phone_number_buffer_size)
{
    EncodedString<maxPhoneLen> encoded;
    eeprom_read_block((void*)encoded.data, (const void*)DataEEPROM::phoneNumberEemem_, maxPhoneLen);

    char decoded[maxPhoneLen] = {0};
    decode(encoded, decoded);
    
    size_t decoded_length = strnlen(decoded, maxPhoneLen);
    if (decoded_length < 4 || decoded_length == maxPhoneLen || decoded_length > static_cast<size_t>(phone_number_buffer_size - 1)) {
        return false;
    }

    for (size_t i = 0; i < decoded_length; ++i) {
      if (!isdigit(decoded[i])) {
          return false;
      }
    }

    return strlcpy(phone_number_buffer, decoded, phone_number_buffer_size) < phone_number_buffer_size;
}

float DataEEPROM::getLastMeasurementKg()
{
    eeprom_busy_wait();
    return eeprom_read_float(&lastMeasurementKgEemem_);
}

float DataEEPROM::getScaleCalibrationValue()
{
    eeprom_busy_wait();
    return eeprom_read_float(&scaleCalibrationValueEemem_);
}

long DataEEPROM::getScaleTareOffset()
{
    long scale_tare_offset;
    eeprom_busy_wait();
    eeprom_read_block((void*)&scale_tare_offset, &scaleTareOffsetEemem_, sizeof(scale_tare_offset));
    return scale_tare_offset;
}

uint16_t DataEEPROM::getInternalAdcReference()
{
    eeprom_busy_wait();
    return eeprom_read_word(&internalAdcReferenceEemem_);
}

uint8_t DataEEPROM::getSMSReportHourOfTheDay() {
    eeprom_busy_wait();
    return eeprom_read_byte(&smsReportHourOfTheDayEemem_);
}

/* Setters */

void DataEEPROM::setLastMeasurementKg(const float last_measurement_kg)
{
    eeprom_busy_wait();
    eeprom_update_float(&lastMeasurementKgEemem_, last_measurement_kg);
}

void DataEEPROM::setScaleCalibrationValue(const float scale_calibration_value)
{
    eeprom_busy_wait();
    eeprom_update_float(&scaleCalibrationValueEemem_, scale_calibration_value);
}

void DataEEPROM::setScaleTareOffset(const long scale_tare_offset)
{
    eeprom_busy_wait();
    eeprom_update_block((void*)&scale_tare_offset, &scaleTareOffsetEemem_, sizeof(scale_tare_offset));
}

void DataEEPROM::setInternalAdcReference(const uint16_t internal_reference)
{
    eeprom_busy_wait();
    eeprom_update_word(&internalAdcReferenceEemem_, internal_reference);
}

void DataEEPROM::setSMSReportHourOfTheDay(const uint8_t hour) {
    eeprom_busy_wait();
    eeprom_update_byte(&smsReportHourOfTheDayEemem_, hour);
}
