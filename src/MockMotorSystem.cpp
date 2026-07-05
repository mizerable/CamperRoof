#include "MockMotorSystem.h"

void MockMotorSystem::init() {
    mm[0].loadFactor = 1.0f;  
    mm[1].loadFactor = 0.9f;  
    mm[2].loadFactor = 1.1f;  
    mm[3].loadFactor = 0.95f; 
}

void MockMotorSystem::getDeltas(int32_t deltas[4]) {
    static int32_t last_positions[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        deltas[i] = mm[i].position - last_positions[i];
        last_positions[i] = mm[i].position;
    }
}

void MockMotorSystem::setThrottles(const int16_t throttles[4]) {
    for (int i = 0; i < 4; i++) {
        mm[i].update(throttles[i]);
    }
}
