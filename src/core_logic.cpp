#include <Arduino.h>
#include "core_logic.h"

CoreLogic::CoreLogic() {
    currentState = SystemState::STATE_WAIT;
    upperLimit = 0;
    fault_clear_timer = 0;
}

void CoreLogic::setInitialState(int32_t positions[4], int32_t limit) {
    upperLimit = limit;
}


void CoreLogic::apply_proportional_throttle(
    bool isLifting, 
    bool overrideLimits, 
    int32_t currentPositions[4],
    int16_t throttles[4]
) {
    int32_t min_pos = currentPositions[0];
    int32_t max_pos = currentPositions[0];

    for (int i = 1; i < 4; i++) {
        if (currentPositions[i] < min_pos) min_pos = currentPositions[i];
        if (currentPositions[i] > max_pos) max_pos = currentPositions[i];
    }

    int32_t midpoint = (min_pos + max_pos) / 2;

    for (int i = 0; i < 4; i++) {
        int throttle = BASE_THROTTLE;
        int32_t deviation = currentPositions[i] - midpoint;

        if (isLifting) {
            throttle = map(deviation, -(MAX_DEVIATION/2), (MAX_DEVIATION/2), MAX_THROTTLE, MIN_THROTTLE);
        } else {
            throttle = map(deviation, -(MAX_DEVIATION/2), (MAX_DEVIATION/2), MIN_THROTTLE, MAX_THROTTLE);
        }

        // Clamp
        if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;
        if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;

        // Direction mapping
        if (!isLifting) {
            throttle = -throttle;
        }

        // Check Individual limits
        if (!overrideLimits) {
            if (isLifting && currentPositions[i] >= upperLimit) {
                throttle = 0;
            }
            if (!isLifting && currentPositions[i] <= 0) {
                throttle = 0;
            }
        }

        throttles[i] = throttle;
    }
}

void CoreLogic::evaluate(
    const ButtonState& btn,
    int32_t currentPositions[4],
    int16_t throttles[4]
) {
    // Default throttles to 0
    for(int i=0; i<4; i++) throttles[i] = 0;

    // 1. Check Fault Condition (Max Deviation)
    int32_t min_pos = currentPositions[0];
    int32_t max_pos = currentPositions[0];
    for (int i = 1; i < 4; i++) {
        if (currentPositions[i] < min_pos) min_pos = currentPositions[i];
        if (currentPositions[i] > max_pos) max_pos = currentPositions[i];
    }
    
    if ((currentState == SystemState::STATE_LIFTING || currentState == SystemState::STATE_LOWERING) 
        && (max_pos - min_pos > MAX_DEVIATION)
        && (!btn.clr)) {
        currentState = SystemState::STATE_FAULT;
    }

    // 2. State Machine
    switch (currentState) {
        case SystemState::STATE_WAIT:
            if (btn.up && !btn.down && !btn.set) {
                currentState = SystemState::STATE_LIFTING;
            } else if (!btn.up && btn.down && !btn.set) {
                currentState = SystemState::STATE_LOWERING;
            } else if (!btn.up && !btn.down && btn.set && !btn.clr) {
                currentState = SystemState::STATE_SET;
            }
            break;

        case SystemState::STATE_LIFTING:
            if (!btn.up) {
                currentState = SystemState::STATE_WAIT;
            } else {
                apply_proportional_throttle(true, btn.clr, currentPositions, throttles);
            }
            break;

        case SystemState::STATE_LOWERING:
            if (!btn.down) {
                currentState = SystemState::STATE_WAIT;
            } else {
                apply_proportional_throttle(false, btn.clr, currentPositions, throttles);
            }
            break;

        case SystemState::STATE_FAULT:
            if (btn.set && btn.clr) {
                fault_clear_timer++;
                if (fault_clear_timer >= (5000 / LOOP_PERIOD_MS)) { // 5 seconds
                    currentState = SystemState::STATE_WAIT;
                    fault_clear_timer = 0;
                }
            } else {
                fault_clear_timer = 0;
            }
            break;

        case SystemState::STATE_SET:
            if (!btn.set) {
                currentState = SystemState::STATE_WAIT;
            } else {
                if (btn.down && !btn.up && !btn.clr) {
                    int32_t max_pos_for_limit = currentPositions[0];
                    for (int i = 1; i < 4; i++) {
                        if (currentPositions[i] > max_pos_for_limit) max_pos_for_limit = currentPositions[i];
                    }
                    upperLimit -= max_pos_for_limit;
                    for (int i = 0; i < 4; i++) currentPositions[i] = 0;
                }
                else if (btn.up && !btn.down && !btn.clr) {
                    upperLimit = max_pos;
                }
            }
            break;
    }
}


