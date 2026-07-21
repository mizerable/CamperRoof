#include <Arduino.h>
#include "app_runner.h"
#include "HardwareMotorSystem.h"
#include "status_led.h"

HardwareMotorSystem hardwareMotors;

#ifndef PIO_UNIT_TESTING

void setup() {
    setup_status_led();
    AppRunner::start(&hardwareMotors);
}

void loop() {
    update_status_led();
    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // PIO_UNIT_TESTING