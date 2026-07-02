#include "HardwareMotorSystem.h"
#include "pcnt_setup.h"
#include "motor_control.h"

void HardwareMotorSystem::init() {
    setup_pcnt();
    setup_motors();
}

void HardwareMotorSystem::updatePositions(int32_t currentPositions[4]) {
    update_pcnt_counts(currentPositions);
}

void HardwareMotorSystem::setPositions(const int32_t positions[4]) {
    pcnt_set_counts(positions);
}

void HardwareMotorSystem::setThrottles(const int16_t throttles[4]) {
    for (int i = 0; i < 4; i++) {
        set_motor_throttle(i, throttles[i]);
    }
}
