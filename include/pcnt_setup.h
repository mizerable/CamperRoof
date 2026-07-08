#pragma once

#include <Arduino.h>

// Initialize the 4 PCNT units for the quadrature encoders
void setup_pcnt();

// Read the incremental change (delta) since the last call. 
// This is completely stateless and relies on hardware quadrature decoding.
void get_and_clear_pcnt_deltas(int32_t deltas[4]);

