#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// Initializes the UART ports for the Cytron MDDS30s
void setup_motors();

// Sets the throttle for a specific motor
// motorIndex: 0 to 3
// throttle: -100 to 100 (negative is reverse, positive is forward, 0 is stop)
// Note: Cytron MDDS30 must be configured for "Simplified Serial Mode" at 9600 baud.
void set_motor_throttle(int motorIndex, int throttle);

// Stops all 4 motors immediately
void stop_all_motors();

#endif // MOTOR_CONTROL_H
