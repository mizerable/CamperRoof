#pragma once

#include "IMotorSystem.h"
#include "system_types.h"

class StallProtectingMotorSystem : public IMotorSystem {
public:
    StallProtectingMotorSystem(IMotorSystem* baseSystem);

    void init() override;
    void getDeltas(int32_t deltas[4]) override;
    void setThrottles(const int16_t throttles[4]) override;

    // Called every loop to inform the protection layer if CLEAR is pressed
    void updateClearButton(bool isClearPressed);
    
    // Allows the display to read the stall status
    MotorState getMotorState(int index) const;

private:
    IMotorSystem* baseSystem;
    
    MotorState currentStates[4];
    
    int32_t delta_history[4][15];
    int16_t throttle_history[4][15];
    int history_idx;
    
    int32_t latest_deltas[4];
    bool clear_pressed;
};
