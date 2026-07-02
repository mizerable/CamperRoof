#ifndef IMOTOR_SYSTEM_H
#define IMOTOR_SYSTEM_H

#include <stdint.h>

class IMotorSystem {
public:
    virtual void init() = 0;
    virtual void updatePositions(int32_t currentPositions[4]) = 0;
    virtual void setPositions(const int32_t positions[4]) = 0;
    virtual void setThrottles(const int16_t throttles[4]) = 0;
    virtual ~IMotorSystem() = default;
};

#endif // IMOTOR_SYSTEM_H
