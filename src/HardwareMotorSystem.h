#ifndef HARDWARE_MOTOR_SYSTEM_H
#define HARDWARE_MOTOR_SYSTEM_H

#include "IMotorSystem.h"

class HardwareMotorSystem : public IMotorSystem {
public:
    void init() override;
    void updatePositions(int32_t currentPositions[4]) override;
    void setPositions(const int32_t positions[4]) override;
    void setThrottles(const int16_t throttles[4]) override;
};

#endif // HARDWARE_MOTOR_SYSTEM_H
