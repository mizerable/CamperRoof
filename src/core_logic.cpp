#include <Arduino.h>
#include "core_logic.h"
#include "state_graph.h"

CoreLogic::CoreLogic() {
    currentNode = &systemGraph.waitNode;
    currentState = SystemState::STATE_WAIT;
    faultClearedFlag = false;
    fault_clear_timer = 0;
    upperLimit = 0;
    stateBeforeMotorSelect = SystemState::STATE_WAIT;
    last_motor_sel_state = false;
    for (int i=0; i<4; i++) {
        lastPositions[i] = 0;
        stallCounters[i] = 0;
        is_bottomed_out[i] = false;
    }
}

void CoreLogic::setInitialState(int32_t positions[4], int32_t limit, const bool bottomedOutFlags[4]) {
    upperLimit = limit;
    for (int i=0; i<4; i++) {
        lastPositions[i] = positions[i];
        stallCounters[i] = 0;
        if (bottomedOutFlags != nullptr) {
            is_bottomed_out[i] = bottomedOutFlags[i];
        } else {
            is_bottomed_out[i] = false;
        }
    }
}

SystemState CoreLogic::getCurrentState() const {
    return currentState;
}

int32_t CoreLogic::getUpperLimit() const {
    return upperLimit;
}

void CoreLogic::getBottomedOutFlags(bool flagsOut[4]) const {
    for(int i=0; i<4; i++) flagsOut[i] = is_bottomed_out[i];
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

bool CoreLogic::hasBottomedOut() const {
    // Only return true if ALL 4 motors are flagged as bottomed out
    return is_bottomed_out[0] && is_bottomed_out[1] && is_bottomed_out[2] && is_bottomed_out[3];
}

void CoreLogic::updateStallDetection(const ButtonState& btn, int32_t currentPositions[4]) {
    for (int i=0; i<4; i++) {
        int32_t delta = abs(currentPositions[i] - lastPositions[i]);
        
        if (currentState == SystemState::STATE_LOWERING) {
            // Ignore intentional stops at soft limit
            if (currentPositions[i] <= 0 && !btn.clr) {
                stallCounters[i] = 0;
            } else if (delta < 20) { // MIN_EXPECTED_DELTA (50 is normal speed, <20 is bogged down)
                stallCounters[i]++;
                if (stallCounters[i] >= 15) { // 0.3 seconds
                    is_bottomed_out[i] = true;
                }
            } else {
                stallCounters[i] = 0;
            }
        } 
        else if (currentState == SystemState::STATE_LIFTING) {
            // Moving UP inherently clears the bottomed out flag
            is_bottomed_out[i] = false;
            stallCounters[i] = 0;
        } else {
            // In WAIT, SET, or FAULT, reset the stall counters so they don't carry over
            stallCounters[i] = 0;
        }
        
        lastPositions[i] = currentPositions[i];
    }
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

        // Individual Limits and Bottom-Out Override
        if (!overrideLimits) {
            if (isLifting && currentPositions[i] >= upperLimit) {
                throttle = 0;
            }
            if (!isLifting && currentPositions[i] <= 0) {
                throttle = 0;
            }
        }
        
        // Safety Bottom-Out Protection ALWAYS overrides throttle (even if overrideLimits is true)
        if (!isLifting && is_bottomed_out[i]) {
            throttle = 0;
        }

        throttles[i] = throttle;
    }
}

void CoreLogic::evaluate(
    const ButtonState& btn,
    int32_t currentPositions[4],
    int16_t throttles[4]
) {
    // 0. Real-time per-motor stall detection
    updateStallDetection(btn, currentPositions);

    // 1. Handle MOTOR_SELECT button edge
    bool motor_sel_rising = btn.motor_sel && !last_motor_sel_state;
    last_motor_sel_state = btn.motor_sel;

    if (motor_sel_rising) {
        if (currentState == SystemState::STATE_WAIT || currentState == SystemState::STATE_FAULT) {
            stateBeforeMotorSelect = currentState;
            currentState = SystemState::STATE_MOTOR1;
            currentNode = &systemGraph.motor1Node;
        }
        else if (currentState == SystemState::STATE_MOTOR1) {
            currentState = SystemState::STATE_MOTOR2;
            currentNode = &systemGraph.motor2Node;
        }
        else if (currentState == SystemState::STATE_MOTOR2) {
            currentState = SystemState::STATE_MOTOR3;
            currentNode = &systemGraph.motor3Node;
        }
        else if (currentState == SystemState::STATE_MOTOR3) {
            currentState = SystemState::STATE_MOTOR4;
            currentNode = &systemGraph.motor4Node;
        }
        else if (currentState == SystemState::STATE_MOTOR4) {
            currentState = stateBeforeMotorSelect;
            if (stateBeforeMotorSelect == SystemState::STATE_WAIT) {
                currentNode = &systemGraph.waitNode;
            } else {
                currentNode = &systemGraph.faultNode;
            }
        }
    }

    // 2. Evaluate Graph Transitions (skip if in manual motor jog state)
    if (currentState != SystemState::STATE_MOTOR1 && 
        currentState != SystemState::STATE_MOTOR2 && 
        currentState != SystemState::STATE_MOTOR3 && 
        currentState != SystemState::STATE_MOTOR4) {
        
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
        case SystemState::STATE_BOTTOMED:
            executeBottomedActions(throttles);
            break;
        case SystemState::STATE_MOTOR1:
            executeMotorJogActions(0, btn, currentPositions, throttles);
            break;
        case SystemState::STATE_MOTOR2:
            executeMotorJogActions(1, btn, currentPositions, throttles);
            break;
        case SystemState::STATE_MOTOR3:
            executeMotorJogActions(2, btn, currentPositions, throttles);
            break;
        case SystemState::STATE_MOTOR4:
            executeMotorJogActions(3, btn, currentPositions, throttles);
            break;
    }
}

void CoreLogic::executeMotorJogActions(int motorIdx, const ButtonState& btn, int32_t pos[4], int16_t throttles[4]) {
    for (int i=0; i<4; i++) throttles[i] = 0;

    if (btn.up && !btn.down) {
        if (btn.clr || pos[motorIdx] < upperLimit) {
            throttles[motorIdx] = 5;
        }
    } else if (btn.down && !btn.up) {
        if (btn.clr || pos[motorIdx] > 0) {
            throttles[motorIdx] = -5;
        }
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

void CoreLogic::executeBottomedActions(int16_t throttles[4]) {
    for(int i=0; i<4; i++) throttles[i] = 0;
}

void CoreLogic::_forceStateForTesting(SystemState state) {
    currentState = state; 
    for (StateNode* node : systemGraph.allNodes) {
        if (node->stateId == state) {
            currentNode = node;
            break;
        }
    }
}
