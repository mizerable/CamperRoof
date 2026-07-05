#include "motor_control.h"
#include "pins.h"

void setup_motors() {
    // Initialize UART ports at 9600 baud for Cytron Simplified Serial mode
    Serial1.begin(9600, SERIAL_8N1, -1, PIN_CYTRON1_TX);
    Serial2.begin(9600, SERIAL_8N1, -1, PIN_CYTRON2_TX);
    
    // Ensure all motors start stopped
    stop_all_motors();
}

void set_motor_throttle(int motorIndex, int throttle) {
    // Clamp throttle to -100 to 100
    if (throttle > 100) throttle = 100;
    if (throttle < -100) throttle = -100;

    uint8_t commandByte = 0;
    bool isRightMotor = (motorIndex % 2 != 0); // Motor 1 and 3 are "Right" channel on their respective Cytrons
    
    if (!isRightMotor) {
        // Left Channel (Motors 0 and 2)
        if (throttle == 0) {
            commandByte = 0; // Stop
        } else if (throttle > 0) {
            commandByte = map(throttle, 1, 100, 1, 63); // CW
        } else {
            commandByte = map(-throttle, 1, 100, 65, 127); // CCW
        }
    } else {
        // Right Channel (Motors 1 and 3)
        if (throttle == 0) {
            commandByte = 128; // Stop
        } else if (throttle > 0) {
            commandByte = map(throttle, 1, 100, 129, 191); // CW
        } else {
            commandByte = map(-throttle, 1, 100, 193, 255); // CCW
        }
    }

    // Send to the correct Cytron
    if (motorIndex < 2) {
        Serial1.write(commandByte);
    } else {
        Serial2.write(commandByte);
    }
}

void stop_all_motors() {
    for (int i = 0; i < 4; i++) {
        set_motor_throttle(i, 0);
    }
}
