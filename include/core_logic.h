#ifndef CORE_LOGIC_H
#define CORE_LOGIC_H

#include "system_types.h"

// Constants
#define MAX_DEVIATION 500
#define BASE_THROTTLE 50
#define MIN_THROTTLE 20
#define MAX_THROTTLE 80
#define LOOP_PERIOD_MS 20 // 50Hz

class CoreLogic {
public:
    CoreLogic();

    // The main evaluation function to be called at 50Hz.
    // Inputs:
    //  - btn: current debounced button states
    //  - currentPositions: current positions of the 4 motors
    // Outputs:
    //  - throttles: the calculated throttles to send to the 4 motors
    //  - fram_write_needed: set to true if the storage needs to be updated this cycle
    void evaluate(
        const ButtonState& btn,
        int32_t currentPositions[4],
        int16_t throttles[4],
        bool& fram_write_needed
    );

    // Getters for current state
    SystemState getCurrentState() const { return currentState; }
    int32_t getUpperLimit() const { return upperLimit; }

    // Setters for initialization from FRAM
    void setInitialState(int32_t positions[4], int32_t limit);


private:
    SystemState currentState;
    int32_t upperLimit;
    int fault_clear_timer;

    void apply_proportional_throttle(
        bool isLifting, 
        bool overrideLimits, 
        int32_t currentPositions[4],
        int16_t throttles[4]
    );


};

#endif // CORE_LOGIC_H
