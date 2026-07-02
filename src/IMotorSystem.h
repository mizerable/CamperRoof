#ifndef IMOTOR_SYSTEM_H
#define IMOTOR_SYSTEM_H

#include <stdint.h>

class IMotorSystem {
public:
    virtual void init() = 0;

    // Read the raw hardware/simulated tick counts (not absolute position)
    virtual void getTicks(int32_t ticks[4]) = 0;

    // Output throttles to the motors (-100 to 100)
    virtual void setThrottles(const int16_t throttles[4]) = 0;
    virtual ~IMotorSystem() = default;
};

#endif // IMOTOR_SYSTEM_H
