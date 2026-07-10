#pragma once

#include <stdint.h>

class IMotorSystem {
public:
    virtual void init() = 0;

    // Read the incremental change since last call
    virtual void getDeltas(int32_t deltas[4]) = 0;

    // Output throttles to the motors (-100 to 100)
    virtual void setThrottles(const int16_t throttles[4]) = 0;
    virtual ~IMotorSystem() = default;
};

