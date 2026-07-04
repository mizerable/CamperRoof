#include "pcnt_setup.h"
#include <ESP32Encoder.h>

// Define pins for the 4 motors (Phase A and Phase B)
const int PCNT_PINS[4][2] = {
    {A2, A13},  // Motor 1 (34, 35)
    {A4, A3},   // Motor 2 (36, 39)
    {A5, MOSI}, // Motor 3 (4, 18)
    {MISO, SDA} // Motor 4 (19, 23)
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
    static int32_t last_counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        int32_t current_count = (int32_t)encoders[i].getCount();
        int32_t delta = current_count - last_counts[i];
        last_counts[i] = current_count;
        currentPositions[i] += delta;
    }
}
