#include "MockMotorSystem.h"

void MockMotorSystem::init() {
    mm[0].loadFactor = 1.0f;  
    mm[1].loadFactor = 0.9f;  
    mm[2].loadFactor = 1.1f;  
    mm[3].loadFactor = 0.95f; 
}

void MockMotorSystem::getTicks(int32_t ticks[4]) {
    for (int i = 0; i < 4; i++) {
        ticks[i] = mm[i].position;
    }
}

void MockMotorSystem::setThrottles(const int16_t throttles[4]) {
    for (int i = 0; i < 4; i++) {
        mm[i].update(throttles[i]);
    }
}
