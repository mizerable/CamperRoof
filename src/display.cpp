#include "display.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// UTRONICS 0.96 OLED typically uses I2C address 0x3C
#define OLED_RESET     -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool setup_display() {
    // Note: Wire.begin() is assumed to be called in main.cpp for the shared I2C bus
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        return false;
    }
    
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Camper Roof System"));
    display.println(F("Initializing..."));
    display.display();
    return true;
}

const char* state_to_string(SystemState state) {
    switch (state) {
        case SystemState::STATE_WAIT: return "WAIT";
        case SystemState::STATE_LIFTING: return "LIFTING";
        case SystemState::STATE_LOWERING: return "LOWERING";
        case SystemState::STATE_FAULT: return "FAULT";
        case SystemState::STATE_SET: return "SET";
        default: return "UNKNOWN";
    }
}

void update_display(const SharedState* state) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print(F("State: "));
    display.println(state_to_string(state->state));
    display.print(F("Top Limit: "));
    display.println(state->upperLimit);
    
    for (int i = 0; i < 4; i++) {
        display.print(F("M"));
        display.print(i + 1);
        display.print(F(": "));
        display.print(state->motors[i].currentPosition);
        display.print(F(" ("));
        display.print(state->motors[i].currentThrottle);
        display.print(F("%)"));
        // Display in 2 columns
        if (i % 2 == 0) {
            display.print(F(" | "));
        } else {
            display.println();
        }
    }
    
    if (state->state == SystemState::STATE_FAULT) {
        display.println(F("! FAULT ! HOLD SET+CLR"));
    }
    
    display.display();
}
