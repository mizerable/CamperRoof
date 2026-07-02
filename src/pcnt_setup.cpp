#include "pcnt_setup.h"
#include <ESP32Encoder.h>

// Define pins for the 4 motors (Phase A and Phase B)
const int PCNT_PINS[4][2] = {
    {34, 35}, // Motor 1
    {36, 39}, // Motor 2
    {4,  18}, // Motor 3
    {19, 23}  // Motor 4
};

ESP32Encoder encoders[4];

void setup_pcnt() {
    ESP32Encoder::useInternalWeakPullResistors = UP;
    
    for (int i = 0; i < 4; i++) {
        encoders[i].setFilter(1023); // 1023 is max filter value for ESP32
        encoders[i].attachFullQuad(PCNT_PINS[i][0], PCNT_PINS[i][1]);
        encoders[i].clearCount();
    }
}

void update_pcnt_counts(int32_t currentPositions[4]) {
    for (int i = 0; i < 4; i++) {
        // Retrieve the 64-bit count and cast to 32-bit (which is plenty)
        currentPositions[i] = (int32_t)encoders[i].getCount();
    }
}

void pcnt_set_counts(const int32_t counts[4]) {
    for (int i = 0; i < 4; i++) {
        encoders[i].setCount(counts[i]);
    }
}
