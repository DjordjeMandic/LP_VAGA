#pragma once

void data_get_last_measurement_kg(float& last_measurement_kg);
void data_get_scale_calibration_value(float& scale_calibration_value);
void data_get_scale_tare_offset(long& scale_tare_offset);
void data_get_internal_adc_reference(uint16_t& internal_reference);

void data_set_last_measurement_kg(const float& last_measurement_kg);
void data_set_scale_calibration_value(const float& scale_calibration_value);
void data_set_scale_tare_offset(const long& scale_tare_offset);
void data_set_internal_adc_reference(const uint16_t& internal_adc_reference);
