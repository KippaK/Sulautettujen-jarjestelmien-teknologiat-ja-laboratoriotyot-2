#include "../include/init.h"

void pin_init() {
    // Sensor reset pin init to high
    DDRB |= 1 << PB6;
    PORTB |= 1 << PB6;
    // SD PINS
}

void sensor_init() {
    Wire.begin();
}

void sd_init() {
    SD.begin();
}