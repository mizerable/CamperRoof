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
    
    int offset = isRightMotor ? 128 : 0;
    
    if (throttle == 0) {
        commandByte = offset; // Stop
    } else if (throttle > 0) {
        commandByte = map(throttle, 1, 100, offset + 1, offset + 63); // CW
    } else {
        commandByte = map(-throttle, 1, 100, offset + 65, offset + 127); // CCW
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
