#include "StallProtectingMotorSystem.h"
#include <stdlib.h> // for abs

StallProtectingMotorSystem::StallProtectingMotorSystem(IMotorSystem* base) 
    : baseSystem(base), history_idx(0), clear_pressed(false) {
    for (int i = 0; i < 4; i++) {
        currentStates[i] = MotorState::OK;
        latest_deltas[i] = 0;
        for (int j = 0; j < 15; j++) {
            delta_history[i][j] = 0;
            throttle_history[i][j] = 0;
        }
    }
}

void StallProtectingMotorSystem::init() {
    baseSystem->init();
}

void StallProtectingMotorSystem::getDeltas(int32_t deltas[4]) {
    baseSystem->getDeltas(deltas);
    for (int i = 0; i < 4; i++) {
        latest_deltas[i] = deltas[i];
    }
}

void StallProtectingMotorSystem::updateClearButton(bool isClearPressed) {
    clear_pressed = isClearPressed;
}

MotorState StallProtectingMotorSystem::getMotorState(int index) const {
    return currentStates[index];
}

void StallProtectingMotorSystem::setThrottles(const int16_t throttles[4]) {
    int16_t final_throttles[4];

    for (int i = 0; i < 4; i++) {
        // Record into history
        delta_history[i][history_idx] = abs(latest_deltas[i]);
        throttle_history[i][history_idx] = abs(throttles[i]);

        if (clear_pressed) {
            currentStates[i] = MotorState::OK;
        }

        // Calculate averages for stall detection
        int32_t old_delta_sum = 0, new_delta_sum = 0;
        int32_t old_throttle_sum = 0, new_throttle_sum = 0;

        for (int j = 0; j < 7; j++) {
            int new_idx = (history_idx - j + 15) % 15;
            int old_idx = (history_idx - j - 8 + 15) % 15;
            
            new_delta_sum += delta_history[i][new_idx];
            old_delta_sum += delta_history[i][old_idx];
            
            new_throttle_sum += throttle_history[i][new_idx];
            old_throttle_sum += throttle_history[i][old_idx];
        }

        int32_t avg_delta_new = new_delta_sum / 7;
        int32_t avg_delta_old = old_delta_sum / 7;
        int32_t avg_throttle_new = new_throttle_sum / 7;
        int32_t avg_throttle_old = old_throttle_sum / 7;

        // If not holding CLEAR, detect stall
        if (!clear_pressed && currentStates[i] == MotorState::OK) {
            // Must have some requested throttle to consider stalling
            bool throttle_ramped_up = (avg_throttle_new >= avg_throttle_old + 20);
            bool throttle_maxed_out = (avg_throttle_new >= 95);
            
            if (avg_throttle_new > 0 && (throttle_ramped_up || throttle_maxed_out)) {
                // Trigger stall if we slowed down OR if we are completely jammed (speed is 0)
                if (avg_delta_new < avg_delta_old || avg_delta_new == 0) {
                    if (throttles[i] > 0) {
                        currentStates[i] = MotorState::STALLED_LIFTING;
                    } else if (throttles[i] < 0) {
                        currentStates[i] = MotorState::STALLED_LOWERING;
                    }
                }
            }
        }

        // Clear stall if commanded opposite direction
        if (currentStates[i] == MotorState::STALLED_LIFTING && throttles[i] < 0) {
            currentStates[i] = MotorState::OK;
        }
        if (currentStates[i] == MotorState::STALLED_LOWERING && throttles[i] > 0) {
            currentStates[i] = MotorState::OK;
        }

        // Override logic
        final_throttles[i] = throttles[i];
        if (currentStates[i] == MotorState::STALLED_LIFTING && final_throttles[i] > 0) {
            final_throttles[i] = 0;
        }
        if (currentStates[i] == MotorState::STALLED_LOWERING && final_throttles[i] < 0) {
            final_throttles[i] = 0;
        }
    }

    history_idx = (history_idx + 1) % 15;
    baseSystem->setThrottles(final_throttles);
}
