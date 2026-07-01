#include <Arduino.h>
#include <unity.h>
#include <Wire.h>
#include "storage.h"
#include "core_logic.h"
#include "display.h"

CoreLogic logic;
ButtonState btn;
int32_t positions[4];
int16_t throttles[4];
bool fram_write_needed;

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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
}

/**
 * @brief Verifies the transition from WAIT to LOWERING.
 * Action: Pressing the DOWN button.
 * Expected: State changes to STATE_LOWERING.
 */
void test_wait_to_lowering(void) {
    btn.down = true; 
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());
}

/**
 * @brief Verifies the transition from WAIT to SET.
 * Action: Pressing the SET button.
 * Expected: State changes to STATE_SET.
 */
void test_wait_to_set(void) {
    btn.set = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // 2. All motors move up together to avoid triggering a max-deviation fault.
    // Motor 0 reaches the upper limit (10,000) exactly.
    positions[0] = 10000;
    positions[1] = 9800; 
    positions[2] = 9800;
    positions[3] = 9800;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());

    // 2. Simulate motors moving down. Motor 0 hits the bottom (0).
    positions[0] = 0;
    positions[1] = 200;
    positions[2] = 200;
    positions[3] = 200;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // 2. Simulate motors being forced way past the 10,000 limit.
    positions[0] = 15000;
    positions[1] = 14800;
    positions[2] = 14800;
    positions[3] = 14800;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // 3. Verify that throttle is still non-zero because the limits were overridden.
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]);
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);

    // 2. Simulate a mechanical failure where Motor 0 stops moving
    positions[0] = 0;
    positions[1] = 600; // Drift is now 600, which is > 500
    
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    positions[0] = 0;
    positions[1] = 600;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // 2. Release the UP button, and press SET + CLEAR
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    
    // 3. Simulate holding the buttons for 4.98 seconds (249 loops)
    for (int i=0; i<249; i++) {
        logic.evaluate(btn, positions, throttles, fram_write_needed);
        // Ensure it is still locked in FAULT
        TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    }
    
    // 4. Simulate the 250th loop (exactly 5.00 seconds)
    logic.evaluate(btn, positions, throttles, fram_write_needed);
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
    
    // 2. Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());

    // 3. Press DOWN to calibrate
    btn.down = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // 4. Verify all position trackers were reset to 0
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, positions[i]);
    }
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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // 3. Press UP to calibrate
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
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
        logic.evaluate(btn, positions, throttles, fram_write_needed);
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
void test_fram_write_needed_flag_optimizations(void) {
    // Idle -> should not write
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_FALSE(fram_write_needed);
    
    // Press UP -> State changed to LIFTING -> should write
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_TRUE(fram_write_needed);
    
    // Release UP -> State changed to WAIT -> should write
    btn.up = false;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_TRUE(fram_write_needed);
    
    // Idle again -> should NOT write
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_FALSE(fram_write_needed);
}

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
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Evaluate again (actually processes LIFTING and computes throttle)
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(0, throttles[0]); 
    
    // Release UP (requires 1 tick to transition from LIFTING back to WAIT)
    btn.up = false;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Press DOWN (transitions to LOWERING)
    btn.down = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Evaluate again (actually processes LOWERING and computes throttle)
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]); // Allowed to move down!
}


/**
 * @brief Verifies Requirement 9: Clearing a fault without fixing the sync instantly faults again.
 * Action: Trigger a FAULT. Clear it to WAIT. Try to move UP without syncing.
 * Expected: State instantly reverts to FAULT.
 */
void test_fault_retriggers_if_not_synced(void) {
    // 1. Force into fault
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    positions[0] = 0;
    positions[1] = 600;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // 2. Clear fault to WAIT
    btn.up = false;
    btn.set = true;
    btn.clr = true;
    for (int i=0; i<250; i++) {
        logic.evaluate(btn, positions, throttles, fram_write_needed);
    }
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());

    // 3. Try to move UP again (positions are still out of sync by 600)
    btn.set = false;
    btn.clr = false;
    btn.up = true;
    
    // First loop transitions WAIT -> LIFTING
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Second loop sees LIFTING + massive deviation -> instantly transitions back to FAULT
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
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
    write_state_to_fram(initial_pos, 0);
    
    // Completely reset the global logic object to match the wiped FRAM
    logic.setInitialState(initial_pos, 0);
    
    // Also reset the global positions array
    for (int i=0; i<4; i++) positions[i] = 0;

    // 3. User pushes UP, lifts to 25000 pulses
    btn.up = true;
    for (int i=0; i<500; i++) {
        logic.evaluate(btn, positions, throttles, fram_write_needed);
        positions[0] += 50; positions[1] += 50; positions[2] += 50; positions[3] += 50;
        if (fram_write_needed) {
            write_state_to_fram(positions, logic.getUpperLimit());
        }
        
        // Update the physical screen occasionally to not flood the I2C bus too much
        if (i % 10 == 0) {
            update_test_display();
        }
        
        delay(2); // Prevent I2C bus flood (simulates real-world loop delay but sped up 10x for testing)
    }
    
    // 4. User sets new Upper Limit
    // First, user releases UP button to stop lifting and enter WAIT state
    btn.up = false;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    if (fram_write_needed) write_state_to_fram(positions, logic.getUpperLimit());
    update_test_display();
    
    // User presses SET to enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    update_test_display();
    
    // User presses UP to lock in the new upper limit
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    if (fram_write_needed) {
        write_state_to_fram(positions, logic.getUpperLimit());
    }
    update_test_display();
    
    // 5. User lowers roof to 5000 pulses
    btn.set = false;
    btn.up = false;
    btn.down = true;
    for (int i=0; i<400; i++) { // Lower by 20000
        logic.evaluate(btn, positions, throttles, fram_write_needed);
        positions[0] -= 50; positions[1] -= 50; positions[2] -= 50; positions[3] -= 50;
        if (fram_write_needed) {
            write_state_to_fram(positions, logic.getUpperLimit());
        }
        
        if (i % 10 == 0) {
            update_test_display();
        }
    }
    
    // 5.5 USER RELEASES BUTTON (Triggers the final FRAM save of the 5000 positions!)
    btn.down = false;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    if (fram_write_needed) {
        write_state_to_fram(positions, logic.getUpperLimit());
    }
    
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
    read_state_from_fram(recovered_positions, &recovered_limit);
    new_logic.setInitialState(recovered_positions, recovered_limit);

    // 8. Verify everything matches reality perfectly!
    TEST_ASSERT_EQUAL(5000, recovered_positions[0]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[1]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[2]);
    TEST_ASSERT_EQUAL(5000, recovered_positions[3]);
    TEST_ASSERT_EQUAL(25000, new_logic.getUpperLimit());
}

void setup() {
    delay(2000); // Wait 2 seconds for the Serial Monitor to connect over USB
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
    RUN_TEST(test_set_down_zeros_positions);
    RUN_TEST(test_set_up_updates_upper_limit);



    RUN_TEST(test_proportional_feedback_convergence);
    RUN_TEST(test_fram_write_needed_flag_optimizations);
    RUN_TEST(test_upper_limit_prevents_up_allows_down);
    RUN_TEST(test_fault_retriggers_if_not_synced);
    RUN_TEST(test_full_lifecycle_with_physical_fram);
    UNITY_END();
}

void loop() {
    // Empty, testing is finished.
}
