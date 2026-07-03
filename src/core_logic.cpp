#include <Arduino.h>
#include "core_logic.h"
#include "state_graph.h"

CoreLogic::CoreLogic() {
    currentState = SystemState::STATE_WAIT;
    currentNode = &systemGraph.waitNode;
    upperLimit = 0;
    fault_clear_timer = 0;
    faultClearedFlag = false;
}

void CoreLogic::setInitialState(int32_t positions[4], int32_t limit) {
    upperLimit = limit;
}

bool CoreLogic::hasDiverged(int32_t currentPositions[4]) const {
    int32_t min_pos = currentPositions[0];
    int32_t max_pos = currentPositions[0];
    for (int i = 1; i < 4; i++) {
        if (currentPositions[i] < min_pos) min_pos = currentPositions[i];
        if (currentPositions[i] > max_pos) max_pos = currentPositions[i];
    }
    return (max_pos - min_pos) > MAX_DEVIATION;
}

bool CoreLogic::canEnterWait(int32_t currentPositions[4]) const {
    if (currentState == SystemState::STATE_FAULT) {
        return faultClearedFlag;
    }
    if (currentState == SystemState::STATE_LIFTING || currentState == SystemState::STATE_LOWERING) {
        return !hasDiverged(currentPositions); // Must not diverge to enter wait from active states
    }
    return true;
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
    // 1. Evaluate Graph Transitions
    for (StateNode* nextNode : currentNode->possibleNext) {
        if (nextNode->signature.matches(btn)) {
            if (nextNode->customCondition == nullptr || nextNode->customCondition(this, btn, currentPositions)) {
                
                // If we are leaving FAULT or entering WAIT, reset the fault flag
                if (currentNode->stateId == SystemState::STATE_FAULT && nextNode->stateId != SystemState::STATE_FAULT) {
                    faultClearedFlag = false;
                    fault_clear_timer = 0;
                }

                currentNode = nextNode;
                currentState = currentNode->stateId;
                break; 
            }
        }
    }

    // 2. Execute ongoing actions based on the current state
    switch (currentState) {
        case SystemState::STATE_WAIT:
            executeWaitActions(throttles);
            break;
        case SystemState::STATE_LIFTING:
            executeLiftingActions(btn.clr, currentPositions, throttles);
            break;
        case SystemState::STATE_LOWERING:
            executeLoweringActions(btn.clr, currentPositions, throttles);
            break;
        case SystemState::STATE_SET:
            executeSetActions(btn, currentPositions, throttles);
            break;
        case SystemState::STATE_FAULT:
            executeFaultActions(btn, throttles);
            break;
    }
}

void CoreLogic::executeWaitActions(int16_t throttles[4]) {
    for(int i=0; i<4; i++) throttles[i] = 0;
}

void CoreLogic::executeLiftingActions(bool overrideLimits, int32_t pos[4], int16_t throttles[4]) {
    apply_proportional_throttle(true, overrideLimits, pos, throttles);
}

void CoreLogic::executeLoweringActions(bool overrideLimits, int32_t pos[4], int16_t throttles[4]) {
    apply_proportional_throttle(false, overrideLimits, pos, throttles);
}

void CoreLogic::executeSetActions(const ButtonState& btn, int32_t pos[4], int16_t throttles[4]) {
    for(int i=0; i<4; i++) throttles[i] = 0;

    if (btn.down && !btn.up && !btn.clr) {
        int32_t max_pos_for_limit = pos[0];
        for (int i = 1; i < 4; i++) {
            if (pos[i] > max_pos_for_limit) max_pos_for_limit = pos[i];
        }
        upperLimit -= max_pos_for_limit;
        for (int i = 0; i < 4; i++) pos[i] = 0;
    }
    else if (btn.up && !btn.down && !btn.clr) {
        int32_t max_pos = pos[0];
        for (int i = 1; i < 4; i++) {
            if (pos[i] > max_pos) max_pos = pos[i];
        }
        upperLimit = max_pos;
    }
}

void CoreLogic::executeFaultActions(const ButtonState& btn, int16_t throttles[4]) {
    for(int i=0; i<4; i++) throttles[i] = 0;

    if (btn.set && btn.clr) {
        fault_clear_timer++;
        if (fault_clear_timer >= (5000 / LOOP_PERIOD_MS)) { // 5 seconds
            faultClearedFlag = true;
        }
    } else {
        // Reset timer if they let go before 5 seconds
        if (!faultClearedFlag) {
            fault_clear_timer = 0;
        }
    }
}


