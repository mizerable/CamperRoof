#pragma once

#include <stdint.h>

// Button state tracking
struct ButtonState {
    bool up;
    bool down;
    bool set;
    bool clr;
    bool motor_sel;
};

// State Machine States
enum class SystemState {
    STATE_WAIT,
    STATE_LIFTING,
    STATE_LOWERING,
    STATE_FAULT,
    STATE_SET,
    STATE_MOTOR1,
    STATE_MOTOR2,
    STATE_MOTOR3,
    STATE_MOTOR4
};

enum class MotorState {
    OK,
    STALLED_LIFTING,
    STALLED_LOWERING
};

struct MotorData {
    int32_t currentPosition;
    int16_t currentThrottle;
    MotorState state;
};

// Global shared state for UI/Display
struct SharedState {
    SystemState state;
    MotorData motors[4];
    int32_t upperLimit;
    ButtonState buttons;
};

