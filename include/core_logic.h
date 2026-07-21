#pragma once

#include "system_types.h"

// Constants
#define MAX_DEVIATION 500
#define BASE_THROTTLE 50
#define MIN_THROTTLE 20
#define MAX_THROTTLE 80
#define LOOP_PERIOD_MS 20 // 50Hz

class StateNode; // Forward declaration

class CoreLogic {
public:
    CoreLogic();

    // The main evaluation function to be called at 50Hz.
    // Inputs:
    //  - btn: current debounced button states
    //  - currentPositions: current positions of the 4 motors
    // Outputs:
    //  - throttles: the calculated throttles to send to the 4 motors
    void evaluate(
        const ButtonState& btn,
        int32_t currentPositions[4],
        int16_t throttles[4]
    );

    // Getters for current state
    SystemState getCurrentState() const;
    int32_t getUpperLimit() const;

    // Setters for initialization from FRAM
    void setInitialState(int32_t currentPositions[4], int32_t upperLimit);


    // State Graph helper methods
    bool hasDiverged(int32_t currentPositions[4]) const;
    bool canEnterWait(int32_t currentPositions[4]) const;

    bool isMotorSelRising() const;
    SystemState getStateBeforeMotorSelect() const;
    void saveStateBeforeMotorSelect(SystemState state);
    void evaluateMotorSelEdge(const ButtonState& btn);

    // For unit testing only
    void _forceStateForTesting(SystemState state);

private:
    SystemState currentState;
    StateNode* currentNode;
    int32_t upperLimit;
    int fault_clear_timer;
    bool faultClearedFlag;
    SystemState stateBeforeMotorSelect;
    bool last_motor_sel_state;
    bool motor_sel_rising_flag;

    // Helper Methods
    int32_t getMinPosition(const int32_t pos[4]) const;
    int32_t getMaxPosition(const int32_t pos[4]) const;
    void clearThrottles(int16_t throttles[4]) const;

    void apply_proportional_throttle(
        bool isLifting, 
        bool overrideLimits, 
        int32_t currentPositions[4],
        int16_t throttles[4]
    );

    // State Execution Methods
    void executeLiftingActions(bool overrideLimits, int32_t pos[4], int16_t throttles[4]);
    void executeLoweringActions(bool overrideLimits, int32_t pos[4], int16_t throttles[4]);
    void executeSetActions(const ButtonState& btn, int32_t pos[4], int16_t throttles[4]);
    void executeFaultActions(const ButtonState& btn, int16_t throttles[4]);
    void executeMotorJogActions(int motorIdx, const ButtonState& btn, int32_t pos[4], int16_t throttles[4]);
};
