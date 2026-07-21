#include <Arduino.h>
#include "app_runner.h"
#include "MockMotorSystem.h"
#include "status_led.h"

MockMotorSystem mockMotors;

#ifndef PIO_UNIT_TESTING

void setup() {
    setup_status_led();
    AppRunner::start(&mockMotors);
}

void loop() {
    update_status_led();
    vTaskDelay(pdMS_TO_TICKS(10));
}

#endif // PIO_UNIT_TESTING
