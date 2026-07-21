#pragma once

#include <Arduino.h>

// Initializes the I2C FRAM
bool setup_storage();

// Read all 4 positions, upper limit, and bottom-out flags from FRAM
void read_state_from_fram(int32_t currentPositions[4], int32_t *upperLimit);

// Write all 4 positions, upper limit, and bottom-out flags to FRAM
// Note: This is fast enough to be called in the 50Hz main loop
void write_state_to_fram(const int32_t currentPositions[4], int32_t upperLimit);

