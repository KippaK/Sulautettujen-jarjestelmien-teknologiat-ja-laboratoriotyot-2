#include <stdint.h>

#include "shared.h"

#define SHT30_ADDRESS 0x44
#define SHT30_SS_HR_MSB 0x2C
#define SHT30_SS_HR_LSB 0x06
#define SHT30_SS_HR_DELAY_MS 15
#define SHT30_MEAS_BYTE_COUNT 6
#define SHT30_SS_HR_MEAS_DELAY_MS 15

unsigned long last_sht30_measurement_ms = 0;

typedef struct {
    uint8_t msb;
    uint8_t lsb;
    uint8_t addr;
    uint8_t delay;
} Sht30_command;

typedef struct {
    uint16_t temp_val;
    uint8_t temp_crc;
    uint16_t humidity_val;
    uint8_t humidity_crc;
} Raw_sensor_data;


typedef struct {
    float temp;
    float humidity;
} Sensor_data;

// SHT30 command for Single-shot, High repeatability (High accuracy)
constexpr Sht30_command SHT30_SS_HR = {
    SHT30_SS_HR_MSB, 
    SHT30_SS_HR_LSB,
    SHT30_ADDRESS,
    SHT30_SS_HR_DELAY_MS 
};

uint8_t calculate_crc(const uint16_t &value);
bool valid_crc(const uint16_t &value, const uint8_t &crc);
Raw_sensor_data read_raw_from_sht30(const uint8_t &address);
Sensor_data convert_raw_sht30_data(const Raw_sensor_data &raw);
Sensor_data get_sht30_meas(const Sht30_command &command);
void send_sht30_measure_command(const Sht30_command &command);
void log_sht30_data_to_serial(const Sensor_data &data);
void reset_sht30();