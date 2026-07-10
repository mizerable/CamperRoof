#pragma once

#include "IMotorSystem.h"

class HardwareMotorSystem : public IMotorSystem {
public:
    void init() override;
    void getDeltas(int32_t deltas[4]) override;
    void setThrottles(const int16_t throttles[4]) override;
};

