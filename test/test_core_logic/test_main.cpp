#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include "storage.h"
#include "core_logic.h"
#include "state_graph.h"
#include "pcnt_setup.h"
#include "display.h"

CoreLogic logic;
ButtonState btn;
int32_t positions[4];
int16_t throttles[4];

void setUp(void) {
    // Reset all inputs to their default states before every single test
    // This ensures no test pollutes the state of another test
    btn = {false, false, false, false};
    for(int i=0; i<4; i++) positions[i] = 0;
    for(int i=0; i<4; i++) throttles[i] = 0;
    
    // Create a fresh instance of the CoreLogic state machine for each test
    logic = CoreLogic();
    // Default the upper travel limit to 10,000 pulses
    logic.setInitialState(positions, 10000); 
}

void tearDown(void) {
    // Unity framework teardown hook (unused)
}

// ---------------------------------------------------------
// 1. STATE TRANSITION TESTS
// ---------------------------------------------------------

/**
 * @brief Verifies that the system boots into the WAIT state by default.
 * Expected: The state machine initializes into SystemState::STATE_WAIT.
 */
void test_initial_state(void) {
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

/**
 * @brief Verifies the transition from WAIT to LIFTING.
 * Action: Pressing the UP button.
 * Expected: State changes to STATE_LIFTING.
 */
void test_wait_to_lifting(void) {
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
}

/**
 * @brief Verifies the transition from WAIT to LOWERING.
 * Action: Pressing the DOWN button.
 * Expected: State changes to STATE_LOWERING.
 */
void test_wait_to_lowering(void) {
    btn.down = true; 
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());
}

/**
 * @brief Verifies the transition from WAIT to SET.
 * Action: Pressing the SET button.
 * Expected: State changes to STATE_SET.
 */
void test_wait_to_set(void) {
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());
}

/**
 * @brief Verifies that invalid button combinations are safely ignored.
 * Action: User mashes UP and DOWN simultaneously (which physically shouldn't happen, but could due to wiring faults).
 * Expected: The system ignores the confusing input and remains safely in STATE_WAIT.
 */
void test_invalid_buttons_ignored(void) {
    btn.up = true;
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

// ---------------------------------------------------------
// 2. LIMIT AND OVERRIDE TESTS
// ---------------------------------------------------------

/**
 * @brief Verifies that a motor stops exactly when it reaches the UPPER_LIMIT.
 * Action: Simulate moving all motors up. Motor 0 exactly hits the 10,000 pulse limit.
 * Expected: Motor 0's throttle is cut to 0. The other motors (which are at 9,800) continue receiving throttle.
 */
void test_upper_limit_stops_motor(void) {
    // 1. Enter LIFTING state
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // 2. All motors move up together to avoid triggering a max-deviation fault.
    // Motor 0 reaches the upper limit (10,000) exactly.
    positions[0] = 10000;
    positions[1] = 9800; 
    positions[2] = 9800;
    positions[3] = 9800;
    logic.evaluate(btn, positions, throttles);
    
    // 3. Verify Motor 0 is stopped, but Motor 1 is still moving.
    TEST_ASSERT_EQUAL(0, throttles[0]);
    TEST_ASSERT_NOT_EQUAL(0, throttles[1]);
}

/**
 * @brief Verifies that a motor stops exactly when it reaches the lower limit (0).
 * Action: Simulate moving all motors down. Motor 0 hits 0 exactly.
 * Expected: Motor 0's throttle is cut to 0.
 */
void test_lower_limit_stops_motor(void) {
    // 1. Enter LOWERING state
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());

    // 2. Simulate motors moving down. Motor 0 hits the bottom (0).
    positions[0] = 0;
    positions[1] = 200;
    positions[2] = 200;
    positions[3] = 200;
    logic.evaluate(btn, positions, throttles);
    
    // 3. Verify Motor 0 is stopped.
    TEST_ASSERT_EQUAL(0, throttles[0]);
}

/**
 * @brief Verifies the manual safety override (Holding UP + CLEAR).
 * Action: Simulate motors moving past the UPPER_LIMIT (10,000) up to 15,000 pulses, but the CLEAR button is held.
 * Expected: The limit checks are bypassed, and throttle is STILL applied to the motors.
 */
void test_override_limits(void) {
    // 1. Enter LIFTING state with CLEAR button held
    btn.up = true;
    btn.clr = true; 
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // 2. Simulate motors being forced way past the 10,000 limit.
    positions[0] = 15000;
    positions[1] = 14800;
    positions[2] = 14800;
    positions[3] = 14800;
    logic.evaluate(btn, positions, throttles);
    
    // 3. Verify that throttle is still non-zero because the limits were overridden.
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]);
}

void test_per_motor_bottom_out_detection(void) {
    for(int i=0; i<4; i++) positions[i] = 5000; // Start high enough to not hit soft limit
    // 1. Start lowering
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    
    // Simulate motors bogging down (delta < 20 per tick) for 15 ticks
    for(int loop=0; loop<15; loop++) {
        for(int i=0; i<4; i++) {
            positions[i] -= 5; 
        }
        logic.evaluate(btn, positions, throttles);
    }
    
    // 2. Verify throttle is forced to 0
    TEST_ASSERT_EQUAL(0, throttles[0]);
    
    // 3. Validate flags set
    bool bottomed_flags[4];
    logic.getBottomedOutFlags(bottomed_flags);
    for(int i=0; i<4; i++) {
        TEST_ASSERT_TRUE(bottomed_flags[i]);
    }
    
    // 4. State should have transitioned to BOTTOMED
    TEST_ASSERT_EQUAL(SystemState::STATE_BOTTOMED, logic.getCurrentState());

    // 5. Test LIFTING clears bottom out flag (Must transition to WAIT first!)
    btn.down = false;
    btn.up = false;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
    
    btn.up = true;
    logic.evaluate(btn, positions, throttles); // Transitions to LIFTING
    logic.evaluate(btn, positions, throttles); // Runs updateStallDetection in LIFTING state
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
    
    // Validate bottomed flags are cleared
    bool cleared_flags[4];
    logic.getBottomedOutFlags(cleared_flags);
    for(int i=0; i<4; i++) {
        TEST_ASSERT_FALSE(cleared_flags[i]);
    }
}

SystemState get_expected_next_state(SystemState current, const ButtonState& b, bool isDiverged, bool isBottomedOut) {
    if (current == SystemState::STATE_WAIT) {
        if (b.up && !b.down && !b.set) return SystemState::STATE_LIFTING;
        if (!b.up && b.down && !b.set) return SystemState::STATE_LOWERING;
        if (!b.up && !b.down && b.set && !b.clr) return SystemState::STATE_SET;
        return SystemState::STATE_WAIT;
    }
    else if (current == SystemState::STATE_LIFTING) {
        if (isDiverged) return SystemState::STATE_FAULT;
        if (!b.up && !b.down && !b.set) return SystemState::STATE_WAIT;
        return SystemState::STATE_LIFTING;
    }
    else if (current == SystemState::STATE_LOWERING) {
        if (isDiverged) return SystemState::STATE_FAULT;
        if (isBottomedOut && !b.up && b.down && !b.set) return SystemState::STATE_BOTTOMED;
        if (!b.up && !b.down && !b.set) return SystemState::STATE_WAIT;
        return SystemState::STATE_LOWERING;
    }
    else if (current == SystemState::STATE_SET) {
        if (!b.up && !b.down && !b.set) return SystemState::STATE_WAIT;
        return SystemState::STATE_SET;
    }
    else if (current == SystemState::STATE_FAULT) {
        return SystemState::STATE_FAULT; // In a single tick test, fault is never cleared.
    }
    else if (current == SystemState::STATE_BOTTOMED) {
        if (!b.up && !b.down && !b.set) return SystemState::STATE_WAIT;
        return SystemState::STATE_BOTTOMED;
    }
    return SystemState::STATE_WAIT;
}

void test_exhaustive_state_transitions(void) {
    CoreLogic exLogic;
    int32_t pos[4] = {5000, 5000, 5000, 5000};
    int16_t throttles[4] = {0,0,0,0};
    
    SystemState all_states[] = {
        SystemState::STATE_WAIT,
        SystemState::STATE_LIFTING,
        SystemState::STATE_LOWERING,
        SystemState::STATE_FAULT,
        SystemState::STATE_SET,
        SystemState::STATE_BOTTOMED
    };
    
    for (SystemState startState : all_states) {
        for (int i=0; i<16; i++) {
            ButtonState b = {
                (i & 1) != 0, // up
                (i & 2) != 0, // down
                (i & 4) != 0, // set
                (i & 8) != 0  // clear
            };
            
            // Re-instantiate to avoid state bleed (like stallCounters accumulating)
            CoreLogic exLogic1;
            exLogic1.setInitialState(pos, 10000, nullptr);
            exLogic1._forceStateForTesting(startState);
            exLogic1.evaluate(b, pos, throttles);
            SystemState expected = get_expected_next_state(startState, b, false, false);
            TEST_ASSERT_EQUAL(expected, exLogic1.getCurrentState());
            
            CoreLogic exLogic2;
            int32_t divPos[4] = {5000, 5000, 5000, 5600};
            exLogic2.setInitialState(pos, 10000, nullptr);
            exLogic2._forceStateForTesting(startState);
            exLogic2.evaluate(b, divPos, throttles);
            expected = get_expected_next_state(startState, b, true, false);
            TEST_ASSERT_EQUAL(expected, exLogic2.getCurrentState());
        }
    }
}

void test_immediate_rebottoming_with_fram(void) {
    // 1. Simulate a reboot with FRAM loaded
    int32_t init_pos[4] = {0, 0, 0, 0};
    bool bottomed_flags[4] = {true, true, true, true};
    
    CoreLogic rebooted_logic;
    rebooted_logic.setInitialState(init_pos, 25000, bottomed_flags);
    
    // Booted up in WAIT state
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, rebooted_logic.getCurrentState());
    
    // 2. User presses DOWN
    btn.up = false;
    btn.down = true;
    btn.set = false;
    btn.clr = false;
    
    // 3. First tick
    int16_t out_throttles[4] = { -1, -1, -1, -1 };
    rebooted_logic.evaluate(btn, init_pos, out_throttles);
    
    // The system evaluates the transition to LOWERING first, but updateStallDetection 
    // preserves the TRUE flags from boot because delta is 0! 
    // Wait, the stall logic checks BEFORE transitions!
    // But then the throttle override kicks in and forces throttles to 0.
    // And because ALL 4 are true, hasBottomedOut is true, and it transitions to BOTTOMED!
    
    // Actually, on the FIRST evaluate from WAIT -> LOWERING, it transitions to LOWERING.
    // On the SECOND evaluate, since it's now in LOWERING, it checks bottomedNode transition!
    rebooted_logic.evaluate(btn, init_pos, out_throttles);
    
    TEST_ASSERT_EQUAL(SystemState::STATE_BOTTOMED, rebooted_logic.getCurrentState());
    
    // Crucially, verify that the motors received exactly ZERO throttle during this entire time
    for (int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, out_throttles[i]);
    }
}

// ---------------------------------------------------------
// 3. FAULT STATE TESTS
// ---------------------------------------------------------

/**
 * @brief Verifies that the system triggers a FAULT if any motor gets stuck or out of sync.
 * Action: Simulate moving UP, but Motor 0 gets stuck at 0 while Motor 1 reaches 600.
 * Expected: The max deviation (600 - 0 = 600) exceeds the MAX_DEVIATION limit (500). 
 *           The system must enter STATE_FAULT and immediately cut ALL throttles to 0.
 */
void test_fault_deviation(void) {
    // 1. Enter LIFTING state
    btn.up = true;
    logic.evaluate(btn, positions, throttles);

    // 2. Simulate a mechanical failure where Motor 0 stops moving
    positions[0] = 0;
    positions[1] = 600; // Drift is now 600, which is > 500
    
    logic.evaluate(btn, positions, throttles);
    
    // 3. Verify system panics into FAULT state
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    
    // 4. Verify ALL motors are instantly stopped
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, throttles[i]);
    }
}

/**
 * @brief Verifies the 5-second hold requirement to escape a FAULT state.
 * Action: Trigger a FAULT. Then hold SET + CLEAR for exactly 250 loops (5 seconds at 50Hz).
 * Expected: The system remains in FAULT for the first 249 loops, and resets to WAIT on the 250th loop.
 */
void test_fault_clear(void) {
    // 1. Force the system into a FAULT
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    positions[0] = 0;
    positions[1] = 600;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // 2. Release the UP button, and press SET + CLEAR
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    
    // 3. Simulate holding the buttons for 4.98 seconds (249 loops)
    for (int i=0; i<249; i++) {
        logic.evaluate(btn, positions, throttles);
        // Ensure it is still locked in FAULT
        TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    }
    
    // 4. Simulate the 250th loop (exactly 5.00 seconds)
    logic.evaluate(btn, positions, throttles);
    // Ensure it is still in FAULT (must release buttons to clear)
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // 5. Release buttons to transition to WAIT
    btn.set = false;
    btn.clr = false;
    logic.evaluate(btn, positions, throttles);
    // Ensure it successfully unlocked and returned to WAIT
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

// ---------------------------------------------------------
// 4. SET STATE TESTS (CALIBRATION)
// ---------------------------------------------------------

/**
 * @brief Verifies that pressing DOWN in the SET state calibrates the bottom limit.
 * Action: Enter SET state. Press DOWN.
 * Expected: All internal position trackers are zeroed out (establishing the new physical baseline).
 */
void test_set_down_zeros_positions(void) {
    // 1. Simulate the roof being partially raised
    positions[0] = 1234;
    positions[1] = 5678;
    
    // Initial limit 10000 (from setUp)
    
    // 2. Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());

    // 3. Press DOWN to calibrate
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    
    // 4. Verify all position trackers were reset to 0
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, positions[i]);
    }
    
    // 5. Verify upper limit did NOT shift down
    // 10000 -> 10000
    TEST_ASSERT_EQUAL(10000, logic.getUpperLimit());
}

void test_set_down_zeros_positions_and_reduces_upper_limit(void) {
    // 1. Simulate motors being at various positions, with upper limit at 10000
    positions[0] = 1000;
    positions[1] = 800;
    positions[2] = 1050; // max pos is 1050
    positions[3] = 950;
    logic.setInitialState(positions, 10000); // Manually inject upper limit of 10000
    
    // 2. Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());

    // 3. Press DOWN to calibrate
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    
    // 4. Verify all position trackers were reset to 0
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, positions[i]);
    }
    
    // 5. Verify upper limit was reduced by the max position (10000 - 1050 = 8950)
    TEST_ASSERT_EQUAL(8950, logic.getUpperLimit());
}

/**
 * @brief Verifies that pressing UP in the SET state calibrates the top limit.
 * Action: Enter SET state. Press UP.
 * Expected: The global UPPER_LIMIT is updated to match the highest motor position currently recorded.
 */
void test_set_up_updates_upper_limit(void) {
    // 1. Simulate the roof being raised unevenly (e.g. manually jogged)
    positions[0] = 1234;
    positions[1] = 5678;
    
    // 2. Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    
    // 3. Press UP to calibrate
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    
    // 4. Verify the upper limit was saved as the highest motor position (5678)
    TEST_ASSERT_EQUAL(5678, logic.getUpperLimit());
}

// ---------------------------------------------------------
// ENTRY POINT (ESP32 HARDWARE RUNNER)
// ---------------------------------------------------------


// ---------------------------------------------------------

// ---------------------------------------------------------
// 6. PROPORTIONAL CONTROL PHYSICS SIMULATION
// ---------------------------------------------------------
void test_proportional_feedback_convergence(void) {
    btn.up = true;
    logic.setInitialState(positions, 50000); // Plenty of room
    
    // Simulate 100 loops of movement.
    // Motor 0 is mathematically simulated to move 1.2x faster for a given PWM.
    // Motor 1 is mathematically simulated to move 0.8x slower for a given PWM.
    for (int loop = 0; loop < 100; loop++) {
        logic.evaluate(btn, positions, throttles);
        TEST_ASSERT_NOT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
        
        positions[0] += (throttles[0] * 1.2); 
        positions[1] += (throttles[1] * 0.8);
        positions[2] += (throttles[2] * 1.0);
        positions[3] += (throttles[3] * 1.0);
    }
    
    // Prove that the algorithm correctly detected the drift and modified the PWM throttles 
    // to force them to converge, keeping the physical difference well under the 500 pulse fault limit.
    int32_t diff = positions[0] - positions[1];
    if (diff < 0) diff = -diff;
    TEST_ASSERT_LESS_THAN(500, diff);
}

// ---------------------------------------------------------
// 7. PERSISTENCE (FRAM) TESTS
// ---------------------------------------------------------
// 8. DIRECTIONAL LIMIT TESTS
// ---------------------------------------------------------
void test_upper_limit_prevents_up_allows_down(void) {
    positions[0] = 10000;
    positions[1] = 10000;
    positions[2] = 10000;
    positions[3] = 10000;
    
    // Try UP (transitions to LIFTING)
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    
    // Evaluate again (actually processes LIFTING and computes throttle)
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(0, throttles[0]); 
    
    // Release UP (requires 1 tick to transition from LIFTING back to WAIT)
    btn.up = false;
    logic.evaluate(btn, positions, throttles);
    
    // Press DOWN (transitions to LOWERING)
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    
    // Evaluate again (actually processes LOWERING and computes throttle)
    logic.evaluate(btn, positions, throttles);
    
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]); // Allowed to move down!
}


// ---------------------------------------------------------
// 9. FAULT OVERRIDE TESTS
// ---------------------------------------------------------
/**
 * @brief Verifies that a user can manually jog the motors to fix a fault by holding CLEAR.
 */
void test_fault_recovery_with_override(void) {
    // 1. Force a fault by manually un-syncing positions
    positions[0] = 0;
    positions[1] = 600;
    btn.up = true;
    logic.evaluate(btn, positions, throttles); // transitions to LIFTING
    logic.evaluate(btn, positions, throttles); // triggers FAULT
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // 2. Clear fault
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    for (int i=0; i<250; i++) {
        logic.evaluate(btn, positions, throttles);
    }
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState()); // Still in fault while holding buttons!

    // 3. Release buttons to transition to WAIT
    btn.set = false;
    btn.clr = false;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());

    // 4. Try to move UP again without CLEAR (positions are still out of sync by 600)
    btn.set = false;
    btn.clr = false;
    btn.up = true;
    
    // First loop transitions WAIT -> LIFTING
    logic.evaluate(btn, positions, throttles);
    
    // Second loop sees LIFTING + massive deviation -> instantly transitions back to FAULT
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    
    // 5. Clear fault again
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    for (int i=0; i<250; i++) {
        logic.evaluate(btn, positions, throttles);
    }
    btn.set = false;
    btn.clr = false;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}
// ---------------------------------------------------------
// 10. PHYSICAL FRAM INTEGRATION TEST
// ---------------------------------------------------------
void update_test_display() {
    SharedState ui_state;
    ui_state.state = logic.getCurrentState();
    ui_state.upperLimit = logic.getUpperLimit();
    ui_state.buttons = btn;
    for (int i=0; i<4; i++) {
        ui_state.motors[i].currentPosition = positions[i];
        ui_state.motors[i].currentThrottle = throttles[i];
    }
    update_display(&ui_state);
}

void test_full_lifecycle_with_physical_fram(void) {
    // 1. Initialize physical I2C and FRAM
    bool fram_ready = setup_storage();
    if (!fram_ready) {
        TEST_IGNORE_MESSAGE("FRAM chip not found on I2C bus!");
        return;
    }
    
    // Initialize OLED Display
    setup_display();

    // 2. Start fresh by overwriting FRAM with zeros
    int32_t initial_pos[4] = {0, 0, 0, 0};
    write_state_to_fram(initial_pos, 0, nullptr);
    
    // Completely reset the global logic object to match the wiped FRAM
    logic.setInitialState(initial_pos, 0);
    
    // Also reset the global positions array
    for (int i=0; i<4; i++) positions[i] = 0;
    
    // Clear the "Initializing" screen immediately
    update_test_display();

    // 3. User pushes UP, lifts to 25000 pulses
    btn.up = true;
    for (int i=0; i<500; i++) {
        logic.evaluate(btn, positions, throttles);
        positions[0] += 50; positions[1] += 50; positions[2] += 50; positions[3] += 50;
        // System writes to FRAM in the loop
        write_state_to_fram(positions, logic.getUpperLimit(), nullptr);
        
        // Update display every 50 loops so it doesn't look frozen
        if (i % 50 == 0) update_test_display();
        
        delay(2); // Prevent I2C bus flood and WDT crashes
    }
    update_test_display();
    
    // 4. User sets new Upper Limit
    // First, user releases UP button to stop lifting and enter WAIT state
    btn.up = false;
    logic.evaluate(btn, positions, throttles);
    write_state_to_fram(positions, logic.getUpperLimit(), nullptr);
    update_test_display();
    
    // User presses SET to enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    update_test_display();
    
    // User presses UP to lock in the new upper limit
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    write_state_to_fram(positions, logic.getUpperLimit(), nullptr);
    update_test_display();
    
    // 5. User releases SET and UP to return to WAIT
    btn.set = false;
    btn.up = false;
    logic.evaluate(btn, positions, throttles);
    update_test_display();

    // 6. User presses DOWN to lower roof to 5000 pulses
    btn.down = true;
    btn.down = true;
    for (int i=0; i<400; i++) { // Lower by 20000
        logic.evaluate(btn, positions, throttles);
        positions[0] -= 50; positions[1] -= 50; positions[2] -= 50; positions[3] -= 50;
        write_state_to_fram(positions, logic.getUpperLimit(), nullptr);
        
        if (i % 50 == 0) update_test_display();
        
        delay(2); // Prevent I2C bus flood and WDT crashes
    }
    update_test_display();
    
    // 5.5 USER RELEASES BUTTON (Triggers the final FRAM save of the 5000 positions!)
    btn.down = false;
    logic.evaluate(btn, positions, throttles);
    write_state_to_fram(positions, logic.getUpperLimit(), nullptr);
    
    // Check local RAM state
    TEST_ASSERT_EQUAL(5000, positions[0]);
    TEST_ASSERT_EQUAL(25000, logic.getUpperLimit());

    // 6. SIMULATE POWER LOSS! Clear RAM completely.
    for(int i=0; i<4; i++) positions[i] = -999;
    CoreLogic new_logic; // Fresh uninitialized instance

    // In a real power loss, the ESP32 hardware AND the FRAM reset completely. 
    // We cannot just call Wire.end() while the FRAM is powered, because it holds SDA low and locks the bus!
    delay(50); // Just let the I2C bus settle normally instead.

    // 7. System Boots back up, reads from physical FRAM
    int32_t recovered_positions[4] = {-1, -1, -1, -1};
    int32_t recovered_limit = -1;
    // On reboot, the system reads from FRAM
    read_state_from_fram(recovered_positions, &recovered_limit, nullptr);
    new_logic.setInitialState(recovered_positions, recovered_limit);

    // 8. Verify everything matches reality perfectly!
    TEST_ASSERT_EQUAL(5000, recovered_positions[0]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[1]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[2]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[3]);
    TEST_ASSERT_EQUAL(25000, new_logic.getUpperLimit());
}

// ---------------------------------------------------------
// 11. MOCK MOTOR PHYSICS INTEGRATION TESTS
// ---------------------------------------------------------

class MockMotor {
public:
    int32_t position = 0;
    float loadFactor = 1.0f;
    float nonLinearity = 1.0f; // Ex: motors might be less efficient at lower throttles
    bool isStuck = false;

    void update(int16_t throttle) {
        if (isStuck) return;
        
        // Simulating physical friction: if throttle is too low, it won't move at all
        if (abs(throttle) < 15) return; 

        // Non-linear response curve: throttle^nonLinearity * loadFactor
        float efficiency = 1.0f;
        if (abs(throttle) < 50) efficiency = 0.8f; // Less efficient at low speeds
        
        position += (throttle * loadFactor * efficiency);
    }
};

void test_simulated_physics_integration(void) {
    // We will simulate 4 motors with completely different physical load characteristics.
    MockMotor mm[4];
    mm[0].loadFactor = 1.0f;  // Normal
    mm[1].loadFactor = 0.7f;  // Heavy load, moves 30% slower for same power
    mm[2].loadFactor = 1.2f;  // Light load, moves 20% faster
    mm[3].loadFactor = 0.9f;  // Slightly heavy

    // Reset global state
    logic.setInitialState(positions, 20000); // Set a 20,000 pulse upper limit

    // 1. SIMULATE LIFTING TO THE TOP
    btn.up = true;
    
    int32_t last_ticks[4] = {0, 0, 0, 0};
    
    // Simulate up to 20,000 loops. We should reach the top way before that.
    bool reached_top = false;
    for (int loop = 0; loop < 20000; loop++) {
        logic.evaluate(btn, positions, throttles);
        
        // Check for faults
        TEST_ASSERT_NOT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
        
        // Apply throttles to physical simulation and calculate deltas
        bool all_stopped = true;
        for (int i = 0; i < 4; i++) {
            mm[i].update(throttles[i]);
            
            int32_t current_ticks = mm[i].position;
            int32_t delta = current_ticks - last_ticks[i];
            positions[i] += delta;
            last_ticks[i] = current_ticks;
            
            if (throttles[i] > 0) all_stopped = false;
        }

        // Verify that deviation NEVER exceeds the fault threshold (500) during active movement!
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                int32_t diff = abs(positions[i] - positions[j]);
                TEST_ASSERT_LESS_THAN(500, diff);
            }
        }
        
        if (all_stopped && loop > 10) { 
            // All motors have reached the upper limit and stopped.
            reached_top = true;
            break; 
        }
    }
    
    TEST_ASSERT_TRUE(reached_top);
    // Verify they all parked right near the limit 
    for(int i=0; i<4; i++) {
        // Due to integer math and physics simulation steps, they might be slightly over/under, but should be extremely close
        TEST_ASSERT_GREATER_OR_EQUAL(19950, positions[i]); 
    }

    // 2. SIMULATE LOWERING BACK TO 0
    btn.up = false;
    logic.evaluate(btn, positions, throttles); // Cycle to WAIT state
    
    btn.down = true;
    bool reached_bottom = false;
    for (int loop = 0; loop < 20000; loop++) {
        logic.evaluate(btn, positions, throttles);
        TEST_ASSERT_NOT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
        
        bool all_stopped = true;
        for (int i = 0; i < 4; i++) {
            mm[i].update(throttles[i]); // Throttles are negative when lowering
            
            int32_t current_ticks = mm[i].position;
            int32_t delta = current_ticks - last_ticks[i];
            positions[i] += delta;
            last_ticks[i] = current_ticks;
            
            if (throttles[i] < 0) all_stopped = false;
        }

        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                int32_t diff = abs(positions[i] - positions[j]);
                TEST_ASSERT_LESS_THAN(500, diff);
            }
        }
        
        if (all_stopped && loop > 10) { 
            reached_bottom = true;
            break; 
        }
    }
    
    TEST_ASSERT_TRUE(reached_bottom);
    for(int i=0; i<4; i++) {
        TEST_ASSERT_LESS_OR_EQUAL(50, positions[i]); // Should park near 0 safely
    }
}

void test_simulated_stuck_motor_fault(void) {
    MockMotor mm[4];
    mm[1].isStuck = true; // Simulating a mechanical jam on motor 1
    
    logic.setInitialState(positions, 20000);
    btn.up = true;
    
    bool faulted = false;
    int32_t last_ticks[4] = {0, 0, 0, 0};
    for (int loop = 0; loop < 1000; loop++) {
        logic.evaluate(btn, positions, throttles);
        
        if (logic.getCurrentState() == SystemState::STATE_FAULT) {
            faulted = true;
            // Verify that once in fault, all throttles are instantly cut to 0
            for(int i=0; i<4; i++) TEST_ASSERT_EQUAL(0, throttles[i]);
            break;
        }
        
        for (int i = 0; i < 4; i++) {
            mm[i].update(throttles[i]);
            
            int32_t current_ticks = mm[i].position;
            int32_t delta = current_ticks - last_ticks[i];
            positions[i] += delta;
            last_ticks[i] = current_ticks;
        }
    }
    
    // The system MUST detect the jam and fault out!
    TEST_ASSERT_TRUE(faulted);
}

void test_fault_state_blocks_all_movement(void) {
    // 1. Force a massive divergence to trigger FAULT state
    positions[0] = 0;
    positions[1] = 1000;
    positions[2] = 500;
    positions[3] = 500;
    
    // Evaluate to enter FAULT
    btn.up = true;
    logic.evaluate(btn, positions, throttles); // Transitions WAIT -> LIFTING
    logic.evaluate(btn, positions, throttles); // Transitions LIFTING -> FAULT
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    
    // Helper lambda to check throttles are 0
    auto assert_zero_throttles = [&]() {
        for(int i=0; i<4; i++) TEST_ASSERT_EQUAL(0, throttles[i]);
    };
    
    // 2. Try lifting
    btn = {false, false, false, false};
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    assert_zero_throttles();
    
    // 3. Try lowering
    btn = {false, false, false, false};
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    assert_zero_throttles();
    
    // 4. Try lifting with CLEAR override
    btn = {false, false, false, false};
    btn.up = true;
    btn.clr = true;
    logic.evaluate(btn, positions, throttles);
    assert_zero_throttles();
    
    // 5. Try lowering with CLEAR override
    btn = {false, false, false, false};
    btn.down = true;
    btn.clr = true;
    logic.evaluate(btn, positions, throttles);
    assert_zero_throttles();
    
    // 6. Ensure state remained FAULT the entire time
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
}

void test_simulated_physics_set_down_zeroes_positions(void) {
    MockMotor mm[4];
    logic.setInitialState(positions, 20000);
    
    // 1. Lift for 100 loops
    btn.up = true;
    int32_t last_ticks[4] = {0, 0, 0, 0};
    for (int loop = 0; loop < 100; loop++) {
        logic.evaluate(btn, positions, throttles);
        for (int i = 0; i < 4; i++) {
            mm[i].update(throttles[i]);
            int32_t current_ticks = mm[i].position;
            int32_t delta = current_ticks - last_ticks[i];
            positions[i] += delta;
            last_ticks[i] = current_ticks;
        }
    }
    
    // Verify we actually moved up
    for (int i=0; i<4; i++) TEST_ASSERT_GREATER_THAN(100, positions[i]);
    
    // 2. Wait state
    btn.up = false;
    logic.evaluate(btn, positions, throttles);
    
    // 3. SET + DOWN to zero positions
    btn.set = true;
    btn.down = false;
    logic.evaluate(btn, positions, throttles); // Enter SET state
    
    int32_t expected_max_pos = positions[0];
    for(int i=1; i<4; i++) {
        if(positions[i] > expected_max_pos) expected_max_pos = positions[i];
    }
    
    btn.down = true;
    logic.evaluate(btn, positions, throttles); // Zero positions
    
    // Verify immediately zeroed
    for (int i=0; i<4; i++) TEST_ASSERT_EQUAL(0, positions[i]);
    
    // Verify upper limit correctly shifted down
    TEST_ASSERT_EQUAL(20000 - expected_max_pos, logic.getUpperLimit());
    
    // 4. Run physics loop a few times and ensure it STAYS zeroed (the bug fix)
    for (int loop = 0; loop < 10; loop++) {
        logic.evaluate(btn, positions, throttles);
        for (int i = 0; i < 4; i++) {
            mm[i].update(throttles[i]);
            int32_t current_ticks = mm[i].position;
            int32_t delta = current_ticks - last_ticks[i];
            positions[i] += delta;
            last_ticks[i] = current_ticks;
        }
        
        // It must remain exactly 0 despite the underlying MockMotor ticks still being > 100
        for (int i=0; i<4; i++) TEST_ASSERT_EQUAL(0, positions[i]);
    }
}

// ---------------------------------------------------------
// 12. GRAPH AMBIGUITY TESTS
// ---------------------------------------------------------

void test_graph_no_ambiguities(void) {
    // We will test every state node
    for (StateNode* node : systemGraph.allNodes) {
        
        // Test all 16 button combinations
        for (int b = 0; b < 16; b++) {
            ButtonState testBtn;
            testBtn.up = (b & 1) != 0;
            testBtn.down = (b & 2) != 0;
            testBtn.set = (b & 4) != 0;
            testBtn.clr = (b & 8) != 0;
            
            // Test both divergence conditions
            for (int div = 0; div <= 1; div++) {
                int32_t pos[4] = {0, 0, 0, 0};
                if (div == 1) pos[0] = 1000; // Force divergence
                
                // Force CoreLogic into the source state so custom conditions (like canEnterWait) 
                // have the correct context to evaluate!
                logic._forceStateForTesting(node->stateId);
                
                int validTransitions = 0;
                
                // Ensure no more than 1 outgoing edge is valid
                for (StateNode* nextNode : node->possibleNext) {
                    if (nextNode->signature.matches(testBtn)) {
                        if (nextNode->customCondition == nullptr || nextNode->customCondition(&logic, testBtn, pos)) {
                            validTransitions++;
                        }
                    }
                }
                
                // Assert that the graph is mathematically unambiguous
                TEST_ASSERT_LESS_OR_EQUAL_MESSAGE(1, validTransitions, "Graph Ambiguity Detected!");
            }
        }
    }
}

void setup() {
    delay(2000); // Give time for Serial monitor to attach
    
    setup_pcnt(); // Initialize hardware PCNT so mock tests don't crash when setCount is called
    
    UNITY_BEGIN();
    
    RUN_TEST(test_initial_state);
    RUN_TEST(test_wait_to_lifting);
    RUN_TEST(test_wait_to_lowering);
    RUN_TEST(test_wait_to_set);
    RUN_TEST(test_invalid_buttons_ignored);
    RUN_TEST(test_upper_limit_stops_motor);
    RUN_TEST(test_lower_limit_stops_motor);
    RUN_TEST(test_override_limits);
    RUN_TEST(test_fault_deviation);
    RUN_TEST(test_fault_clear);
    extern void test_set_down_zeros_positions_and_reduces_upper_limit(void);
    RUN_TEST(test_set_down_zeros_positions_and_reduces_upper_limit);
    RUN_TEST(test_set_up_updates_upper_limit);



    RUN_TEST(test_proportional_feedback_convergence);
    RUN_TEST(test_upper_limit_prevents_up_allows_down);
    RUN_TEST(test_fault_recovery_with_override);
    RUN_TEST(test_full_lifecycle_with_physical_fram);
    RUN_TEST(test_simulated_physics_integration);
    RUN_TEST(test_simulated_stuck_motor_fault);
    
    extern void test_fault_state_blocks_all_movement(void);
    RUN_TEST(test_fault_state_blocks_all_movement);

    extern void test_simulated_physics_set_down_zeroes_positions(void);
    RUN_TEST(test_simulated_physics_set_down_zeroes_positions);

    extern void test_fault_mode_allows_manual_jogging_updates(void);
    RUN_TEST(test_fault_mode_allows_manual_jogging_updates);

    extern void test_pcnt_wrap_around_math(void);
    RUN_TEST(test_pcnt_wrap_around_math);

    extern void test_set_state_combinations(void);
    RUN_TEST(test_set_state_combinations);

    extern void test_wait_up_and_down_ignored(void);
    RUN_TEST(test_wait_up_and_down_ignored);
    
    extern void test_wait_set_and_down_ignored(void);
    RUN_TEST(test_wait_set_and_down_ignored);
    
    extern void test_lifting_down_ignored(void);
    RUN_TEST(test_lifting_down_ignored);
    
    extern void test_set_up_and_down_ignored(void);
    RUN_TEST(test_set_up_and_down_ignored);
    
    extern void test_override_limits_lowering(void);
    RUN_TEST(test_override_limits_lowering);
    
    extern void test_per_motor_bottom_out_detection(void);
    RUN_TEST(test_per_motor_bottom_out_detection);
    
    extern void test_immediate_rebottoming_with_fram(void);
    RUN_TEST(test_immediate_rebottoming_with_fram);
    
    extern void test_exhaustive_state_transitions(void);
    RUN_TEST(test_exhaustive_state_transitions);
    
    extern void test_graph_no_ambiguities(void);
    RUN_TEST(test_graph_no_ambiguities);

    UNITY_END();
}

void test_wait_up_and_down_ignored(void) {
    btn.up = true;
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_wait_set_and_down_ignored(void) {
    btn.set = true;
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_lifting_down_ignored(void) {
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
}

void test_set_up_and_down_ignored(void) {
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    btn.up = true;
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(10000, logic.getUpperLimit()); // Limit unchanged
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());
}

void test_override_limits_lowering(void) {
    btn.down = true;
    btn.clr = true; 
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());

    positions[0] = -500;
    positions[1] = -500;
    positions[2] = -500;
    positions[3] = -500;
    logic.evaluate(btn, positions, throttles);
    
    // Should still have throttle despite being < 0
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]);
}

void test_fault_mode_allows_manual_jogging_updates(void) {
    // Enter fault mode
    btn.up = true;
    logic.evaluate(btn, positions, throttles);
    positions[0] = 0;
    positions[1] = 600;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    
    // Simulate manual jogging via Cytron buttons (PCNT still updates positions)
    positions[0] = 600; // jogged back into sync manually
    logic.evaluate(btn, positions, throttles);
    
    // Verify positions remain updated (CoreLogic didn't overwrite them)
    TEST_ASSERT_EQUAL(600, positions[0]);
    TEST_ASSERT_EQUAL(600, positions[1]);
    
    // Now we can clear fault since they are in sync
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    for (int i=0; i<250; i++) logic.evaluate(btn, positions, throttles);
    btn.set = false;
    btn.clr = false;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_pcnt_wrap_around_math(void) {
    // We can test the 16-bit wrap-around logic explicitly
    // If a motor moves from 32760 to 32770, a 16-bit register wraps to -32766
    int16_t last_count = 32760;
    int16_t current_count = -32766; // 32770 in 16-bit signed
    int16_t delta_16 = current_count - last_count;
    int32_t delta_32 = delta_16; // sign extend
    TEST_ASSERT_EQUAL(10, delta_32);
    
    // Lowering wrap around
    last_count = -32760;
    current_count = 32766; // -32770 in 16-bit signed
    delta_16 = current_count - last_count;
    delta_32 = delta_16;
    TEST_ASSERT_EQUAL(-10, delta_32);
}

void test_set_state_combinations(void) {
    // 1. User enters SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());
    
    // 2. User presses DOWN to zero bottom limit
    btn.down = true;
    logic.evaluate(btn, positions, throttles);
    for(int i=0; i<4; i++) TEST_ASSERT_EQUAL(0, positions[i]);
    
    // 3. User lifts roof by 5000 pulses while out of SET STATE
    btn.down = false;
    btn.set = false;
    logic.evaluate(btn, positions, throttles); // Back to WAIT
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
    
    btn.up = true;
    logic.evaluate(btn, positions, throttles); // LIFTING
    positions[0] = 5000; positions[1] = 5000; positions[2] = 5000; positions[3] = 5000;
    logic.evaluate(btn, positions, throttles);
    
    btn.up = false;
    logic.evaluate(btn, positions, throttles); // Back to WAIT
    
    btn.set = true;
    logic.evaluate(btn, positions, throttles); // Enter SET
    btn.up = true;
    logic.evaluate(btn, positions, throttles); // Set TOP limit
    
    TEST_ASSERT_EQUAL(5000, logic.getUpperLimit());
}

void loop() {
    // Empty, testing is finished.
}

