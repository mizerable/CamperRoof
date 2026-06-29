#ifndef PCNT_SETUP_H
#define PCNT_SETUP_H

#include <Arduino.h>
#include "driver/pcnt.h"

// Initialize the 4 PCNT units for the quadrature encoders
void setup_pcnt();

// Read the current absolute position, safely handling 16-bit rollovers
// Must be called frequently (e.g. at 50Hz) to prevent multiple rollovers between reads.
void update_pcnt_counts(int32_t currentPositions[4]);

#endif // PCNT_SETUP_H
