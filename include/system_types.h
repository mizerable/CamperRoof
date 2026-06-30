#ifndef SYSTEM_TYPES_H
#define SYSTEM_TYPES_H

#include <stdint.h>

// Button state tracking
struct ButtonState {
    bool up;
    bool down;
    bool set;
    bool clr;
};

// State Machine States
enum class SystemState {
    STATE_WAIT,
    STATE_LIFTING,
    STATE_LOWERING,
    STATE_FAULT,
    STATE_SET
};

struct MotorData {
    int32_t currentPosition;
    int16_t currentThrottle;
};

// Global shared state for UI/Display
struct SharedState {
    SystemState state;
    MotorData motors[4];
    int32_t upperLimit;
    ButtonState buttons;
};

#endif // SYSTEM_TYPES_H
