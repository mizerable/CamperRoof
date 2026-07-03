#ifndef CORE_LOGIC_H
#define CORE_LOGIC_H

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
    void getBottomedOutFlags(bool flagsOut[4]) const;

    // Setters for initialization from FRAM
    void setInitialState(int32_t currentPositions[4], int32_t upperLimit, const bool bottomedOutFlags[4] = nullptr);


    // State Graph helper methods
    bool hasDiverged(int32_t currentPositions[4]) const;
    bool canEnterWait(int32_t currentPositions[4]) const;
    bool hasBottomedOut() const;
    void updateStallDetection(const ButtonState& btn, int32_t currentPositions[4]);

    // For unit testing only
    void _forceStateForTesting(SystemState state);

private:
    SystemState currentState;
    StateNode* currentNode;
    int32_t upperLimit;
    int fault_clear_timer;
    bool faultClearedFlag;

    // Bottom-Out Detection State
    int32_t lastPositions[4];
    int stallCounters[4];
    bool is_bottomed_out[4];

    void apply_proportional_throttle(
        bool isLifting, 
        bool overrideLimits, 
        int32_t currentPositions[4],
        int16_t throttles[4]
    );

    // Isolated Action Handlers
    void executeWaitActions(int16_t throttles[4]);
    void executeLiftingActions(bool overrideLimits, int32_t currentPositions[4], int16_t throttles[4]);
    void executeLoweringActions(bool overrideLimits, int32_t currentPositions[4], int16_t throttles[4]);
    void executeSetActions(const ButtonState& btn, int32_t currentPositions[4], int16_t throttles[4]);
    void executeFaultActions(const ButtonState& btn, int16_t throttles[4]);
    void executeBottomedActions(int16_t throttles[4]);
};

#endif // CORE_LOGIC_H
