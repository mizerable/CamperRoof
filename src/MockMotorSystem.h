#ifndef MOCK_MOTOR_SYSTEM_H
#define MOCK_MOTOR_SYSTEM_H

#include "IMotorSystem.h"
#include <Arduino.h>

class MockMotor {
public:
    int32_t position = 0;
    float loadFactor = 1.0f;
    float nonLinearity = 1.0f;
    bool isStuck = false;

    void update(int16_t throttle) {
        if (isStuck) return;
        
        // Simulating physical friction: if throttle is too low, it won't move at all
        if (abs(throttle) < 4) return; 

        // Non-linear response curve: throttle^nonLinearity * loadFactor
        float efficiency = 1.0f;
        if (abs(throttle) < 50) efficiency = 0.8f; // Less efficient at low speeds
        
        position += (throttle * loadFactor * efficiency);
    }
};

class MockMotorSystem : public IMotorSystem {
private:
    MockMotor mm[4];
public:
    void init() override;
    void getDeltas(int32_t deltas[4]) override;
    void setThrottles(const int16_t throttles[4]) override;
};

#endif // MOCK_MOTOR_SYSTEM_H
