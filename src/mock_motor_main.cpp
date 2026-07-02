#include <Arduino.h>
#include "app_runner.h"
#include "MockMotorSystem.h"

MockMotorSystem mockMotors;

#ifndef PIO_UNIT_TESTING

void setup() {
    AppRunner::start(&mockMotors);
}

void loop() {
    // Empty, FreeRTOS handles the tasks
    vTaskDelete(NULL);
}

#endif // PIO_UNIT_TESTING
