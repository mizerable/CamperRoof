#ifndef STORAGE_H
#define STORAGE_H

#include <Arduino.h>

// Initializes the I2C FRAM
bool setup_storage();

// Read all 4 positions and the upper limit from FRAM
void read_state_from_fram(int32_t currentPositions[4], int32_t *upperLimit);

// Write all 4 positions and the upper limit to FRAM
// Note: This is fast enough to be called in the 50Hz main loop
void write_state_to_fram(const int32_t currentPositions[4], int32_t upperLimit);

#endif // STORAGE_H
