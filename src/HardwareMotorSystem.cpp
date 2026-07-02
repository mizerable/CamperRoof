#include "HardwareMotorSystem.h"
#include "pcnt_setup.h"
#include "motor_control.h"

void HardwareMotorSystem::init() {
    setup_pcnt();
    setup_motors();
}

void HardwareMotorSystem::getTicks(int32_t ticks[4]) {
    update_pcnt_counts(ticks);
}

void HardwareMotorSystem::setThrottles(const int16_t throttles[4]) {
    for (int i = 0; i < 4; i++) {
        set_motor_throttle(i, throttles[i]);
    }
}
