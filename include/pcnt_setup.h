#ifndef PCNT_SETUP_H
#define PCNT_SETUP_H

#include <Arduino.h>
// Initialize the 4 PCNT units for the quadrature encoders
void setup_pcnt();

// Read the current absolute position
void update_pcnt_counts(int32_t currentPositions[4]);

#endif // PCNT_SETUP_H
