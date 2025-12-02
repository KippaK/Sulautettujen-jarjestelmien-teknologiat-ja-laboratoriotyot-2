#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "../include/sht30.h"
#include "../include/init.h"
#include "../include/sd_module.h"

// Log file info
#define STATUS_FILE_NAME "status.ndjson"
#define LOG_FILE_NAME "log.ndjson"
File log_file;
File status_file;

#define LOG_BUFFER_SIZE 5
#define MAX_LOG_STRING_VALUE_LENGTH 64
#define MAX_LOG_NAME_LENGTH 10


// PINS
/*
MCU_RST     PC6     BY DEFAULT 

SD_CMD      PB3
SD_DAT      PB4
SD_CLK      PB5

MCU_RX      PD0
MCU_TX      PD1
MCU_CD      PD2
MCU_WP      PD3

SENSOR_SCL  PC5
SENSOR_SDA  PC4
SENSOR_RST  PB7 

*/

void setup() {
    pin_init();
    // sd_init();
    sensor_init();

    send_sht30_measure_command(SHT30_SS_HR);
    delay(SHT30_SS_HR.delay);
}

void loop() {
    Sensor_data data = get_sht30_meas(SHT30_SS_HR);
    log_sht30_data_to_serial(data);
}