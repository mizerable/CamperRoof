#include "storage.h"
#include <Adafruit_FRAM_I2C.h>

Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

#define FRAM_ADDR_POSITIONS 0x00
#define FRAM_ADDR_LIMIT     0x10
#define FRAM_ADDR_MAGIC     0x14

#define MAGIC_VALUE 0xCAFE

void recoverI2CBus() {
    pinMode(23, INPUT_PULLUP); // SDA
    pinMode(22, OUTPUT);       // SCL
    digitalWrite(22, HIGH);
    
    // If SDA is low, the bus is stuck. Clock SCL up to 9 times to free it.
    for (int i = 0; i < 9; i++) {
        if (digitalRead(23) == HIGH) {
            break; // Bus is free!
        }
        digitalWrite(22, LOW);
        delayMicroseconds(5);
        digitalWrite(22, HIGH);
        delayMicroseconds(5);
    }
    
    // Generate a STOP condition
    digitalWrite(23, LOW);
    pinMode(23, OUTPUT);
    delayMicroseconds(5);
    digitalWrite(22, HIGH);
    delayMicroseconds(5);
    digitalWrite(23, HIGH);
    delayMicroseconds(5);
}

// Initialize the I2C FRAM
bool setup_storage() {
    recoverI2CBus();

    Wire.begin();
    Wire.setTimeOut(100); // Prevent infinite hang if SCL/SDA are physically shorted to GND
    
    // Drop I2C clock to 50kHz for stability
    Wire.setClock(50000);

    Serial.println("\n--- I2C BUS SCAN ---");
    int devices = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found I2C device at 0x%02X\n", addr);
            devices++;
        }
    }
    if (devices == 0) {
        Serial.println("No I2C devices found! The bus is completely dead!");
    }
    Serial.println("--------------------");

    // Attempt to initialize I2C FRAM
    if (!fram.begin()) {
        return false; // FRAM not found
    }

    uint16_t magic = 0;
    fram.read(FRAM_ADDR_MAGIC, (uint8_t*)&magic, sizeof(magic));

    if (magic != MAGIC_VALUE) {
        // First boot or uninitialized memory, set all to 0
        int32_t zeros[4] = {0, 0, 0, 0};
        write_state_to_fram(zeros, 0);
        magic = MAGIC_VALUE;
        fram.write(FRAM_ADDR_MAGIC, (uint8_t*)&magic, sizeof(magic));
    }

    return true;
}

void read_state_from_fram(int32_t currentPositions[4], int32_t *upperLimit) {
    fram.read(FRAM_ADDR_POSITIONS, (uint8_t*)currentPositions, sizeof(int32_t) * 4);
    fram.read(FRAM_ADDR_LIMIT, (uint8_t*)upperLimit, sizeof(int32_t));
}

void write_state_to_fram(const int32_t currentPositions[4], int32_t upperLimit) {
    fram.write(FRAM_ADDR_POSITIONS, (uint8_t*)currentPositions, sizeof(int32_t) * 4);
    fram.write(FRAM_ADDR_LIMIT, (uint8_t*)&upperLimit, sizeof(int32_t));
}
