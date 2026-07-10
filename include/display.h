#pragma once

#include <Arduino.h>
#include "system_types.h"

// Setup I2C SSD1306 Display
bool setup_display();

// Update the screen with the current state (called on Core 0)
void update_display(const SharedState* state);

