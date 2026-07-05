#include "pcnt_setup.h"
#include "pins.h"
#include <ESP32Encoder.h>

static ESP32Encoder encoders[4];

void setup_pcnt() {
    ESP32Encoder::useInternalWeakPullResistors = UP;

    const int PCNT_PINS[4][2] = {
        {PIN_M1_PHASE_A, PIN_M1_PHASE_B},
        {PIN_M2_PHASE_A, PIN_M2_PHASE_B},
        {PIN_M3_PHASE_A, PIN_M3_PHASE_B},
        {PIN_M4_PHASE_A, PIN_M4_PHASE_B}
    };
    
    for (int i = 0; i < 4; i++) {
        encoders[i].setFilter(1023); // 1023 is max filter value for ESP32
        encoders[i].attachFullQuad(PCNT_PINS[i][0], PCNT_PINS[i][1]);
        encoders[i].clearCount();
    }
}

void get_and_clear_pcnt_deltas(int32_t deltas[4]) {
    static int32_t last_counts[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        // ESP32Encoder internally handles the 64-bit rollover math safely
        int32_t current_count = (int32_t)encoders[i].getCount();
        
        int32_t delta = current_count - last_counts[i];
        last_counts[i] = current_count;
        
        deltas[i] = delta; 
    }
}
