#include "MockMotorSystem.h"

void MockMotorSystem::init() {
    mm[0].loadFactor = 1.0f;  
    mm[1].loadFactor = 0.9f;  
    mm[2].loadFactor = 1.1f;  
    mm[3].loadFactor = 0.95f; 
}

void MockMotorSystem::updatePositions(int32_t currentPositions[4]) {
    for (int i = 0; i < 4; i++) {
        currentPositions[i] = mm[i].position;
    }
}

void MockMotorSystem::setPositions(const int32_t positions[4]) {
    for (int i = 0; i < 4; i++) {
        mm[i].position = positions[i];
    }
}

void MockMotorSystem::setThrottles(const int16_t throttles[4]) {
    for (int i = 0; i < 4; i++) {
        mm[i].update(throttles[i]);
    }
}
