#include "storage.h"
#include <Adafruit_FRAM_I2C.h>

Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

#define FRAM_ADDR_POSITIONS 0x00
#define FRAM_ADDR_LIMIT 0x10
#define FRAM_ADDR_MAGIC 0x14
#define FRAM_ADDR_BOTTOMED 0x16

#define MAGIC_VALUE 0xCAFF

// Initialize the I2C FRAM
bool setup_storage() {
  Wire.begin();
  Wire.setTimeOut(
      100); // Prevent infinite hang if SCL/SDA are physically shorted to GND

  // Drop I2C clock to 50kHz for stability on breadboard
  Wire.setClock(50000);

  // Attempt to initialize I2C FRAM at address 0x50
  if (!fram.begin(0x50)) {
    Serial.println("FRAM begin failed at 0x50");
    return false; // FRAM not found
  }

  uint16_t magic = 0;
  fram.read(FRAM_ADDR_MAGIC, reinterpret_cast<uint8_t *>(&magic), sizeof(magic));

    if (magic != MAGIC_VALUE) {
    // First boot or uninitialized memory, set all to 0
    int32_t zeros[4] = {0, 0, 0, 0};
    bool false_flags[4] = {false, false, false, false};
    write_state_to_fram(zeros, 0, false_flags);
    magic = MAGIC_VALUE;
    fram.write(FRAM_ADDR_MAGIC, reinterpret_cast<uint8_t *>(&magic), sizeof(magic));
  }

  return true;
}

void read_state_from_fram(int32_t currentPositions[4], int32_t *upperLimit, bool bottomedOutFlags[4]) {
  fram.read(FRAM_ADDR_POSITIONS, reinterpret_cast<uint8_t *>(currentPositions),
            sizeof(int32_t) * 4);
  fram.read(FRAM_ADDR_LIMIT, reinterpret_cast<uint8_t *>(upperLimit), sizeof(int32_t));
  if (bottomedOutFlags != nullptr) {
      fram.read(FRAM_ADDR_BOTTOMED, reinterpret_cast<uint8_t *>(bottomedOutFlags), sizeof(bool) * 4);
  }
}

void write_state_to_fram(const int32_t currentPositions[4],
                         int32_t upperLimit, const bool bottomedOutFlags[4]) {
  fram.write(FRAM_ADDR_POSITIONS, const_cast<uint8_t*>(reinterpret_cast<const uint8_t *>(currentPositions)),
             sizeof(int32_t) * 4);
  fram.write(FRAM_ADDR_LIMIT, const_cast<uint8_t*>(reinterpret_cast<const uint8_t *>(&upperLimit)), sizeof(int32_t));
  if (bottomedOutFlags != nullptr) {
      fram.write(FRAM_ADDR_BOTTOMED, const_cast<uint8_t*>(reinterpret_cast<const uint8_t *>(bottomedOutFlags)), sizeof(bool) * 4);
  }
}
