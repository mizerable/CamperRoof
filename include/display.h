#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>

// Matches the enums in main
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

struct SharedState {
    SystemState state;
    MotorData motors[4];
    int32_t upperLimit;
};

// Setup I2C SSD1306 Display
bool setup_display();

// Update the screen with the current state (called on Core 0)
void update_display(const SharedState* state);

#endif // DISPLAY_H
