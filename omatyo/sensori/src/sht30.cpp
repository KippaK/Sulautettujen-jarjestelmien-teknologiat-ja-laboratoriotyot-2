#include <Wire.h>

#include "../include/sht30.h"
#include "../include/shared.h"

uint8_t calculate_crc(const uint16_t &value) {
    uint8_t crc = 0xFF;

    uint8_t bytes[2];
    bytes[0] = (value >> 8) & 0xFF;  // MSB
    bytes[1] = value & 0xFF;         // LSB

    for (int i = 0; i < 2; i++) {
        crc ^= bytes[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool valid_crc(const uint16_t &value, const uint8_t &crc) {
    return calculate_crc(value) == crc;
}

Raw_sensor_data read_raw_from_sht30(const uint8_t &address) {
    Wire.requestFrom(address, SHT30_MEAS_BYTE_COUNT);
    Raw_sensor_data raw_data;
    if (Wire.available() == SHT30_MEAS_BYTE_COUNT) {
        raw_data.temp_val = (Wire.read() << 8) | Wire.read();
        raw_data.temp_crc = Wire.read();
        raw_data.humidity_val = (Wire.read() << 8) | Wire.read();
        raw_data.humidity_crc = Wire.read();
    }
    return raw_data;
}

Sensor_data convert_raw_sht30_data(const Raw_sensor_data &raw) {
    Sensor_data data;
    data.temp = -45 + 175.0 * (float)raw.temp_val / 65535.0;
    data.humidity = 100.0 * (float)raw.humidity_val / 65535.0;
    return data;
}

Sensor_data get_sht30_meas(const Sht30_command &command) {
    while (millis() - last_sht30_measurement_ms < command.delay) { delay(1); }
    Raw_sensor_data raw;
    for (;;) {
        raw = read_raw_from_sht30(command.addr);
        if (
            valid_crc(raw.humidity_val, raw.humidity_crc) && 
            valid_crc(raw.temp_val, raw.temp_crc)
        ) { break; }
        send_sht30_measure_command(command);
    }
    return convert_raw_sht30_data(raw);
}

void send_sht30_measure_command(const Sht30_command &command) {
    Wire.beginTransmission(command.addr);
    Wire.write(command.msb);
    Wire.write(command.lsb);
    Wire.endTransmission();
    last_sht30_measurement_ms = millis();
}

void log_sht30_data_to_serial(const Sensor_data &data) {
    if (~Serial) {
        Serial.begin(SERIAL_SPEED);
        delay(10);
    }
    Serial.print("Temp: ");
    Serial.print(data.temp);
    Serial.print(" °C, Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %");
}

void reset_sht30() {
    PORTB &= ~(1 << PB6);
    delay(1);
    PORTB |= 1 << PB6;
}