#include <Arduino.h>
#include <unity.h>
#include "core_logic.h"

CoreLogic logic;
ButtonState btn;
int32_t positions[4];
int16_t throttles[4];
bool fram_write_needed;

void setUp(void) {
    // Reset inputs for each test
    btn = {false, false, false, false};
    for(int i=0; i<4; i++) positions[i] = 0;
    for(int i=0; i<4; i++) throttles[i] = 0;
    
    // Create a fresh instance for each test
    logic = CoreLogic();
    logic.setInitialState(positions, 10000); // Default upper limit 10000
}

void tearDown(void) {
    // clean stuff up here
}

void test_initial_state(void) {
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_wait_to_lifting(void) {
    btn.up = true; // Press UP
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());
}

void test_wait_to_lowering(void) {
    btn.down = true; // Press DOWN
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());
}

void test_wait_to_set(void) {
    btn.set = true; // Press SET
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());
}

void test_invalid_buttons_ignored(void) {
    // Press UP and DOWN at the same time
    btn.up = true;
    btn.down = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    // Should remain in WAIT because of the !btn.down check
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_upper_limit_stops_motor(void) {
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // All motors move up together to avoid FAULT state (Max deviation 500)
    // Motor 0 reaches upper limit (10000) exactly
    positions[0] = 10000;
    positions[1] = 9800; 
    positions[2] = 9800;
    positions[3] = 9800;

    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Motor 0 should have 0 throttle because it hit the limit
    TEST_ASSERT_EQUAL(0, throttles[0]);
    // Other motors should still have throttle because they are at 9800
    TEST_ASSERT_NOT_EQUAL(0, throttles[1]);
}

void test_lower_limit_stops_motor(void) {
    btn.down = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LOWERING, logic.getCurrentState());

    // Motor 0 reaches lower limit (0)
    positions[0] = 0;
    positions[1] = 200;
    positions[2] = 200;
    positions[3] = 200;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Motor 0 should have 0 throttle
    TEST_ASSERT_EQUAL(0, throttles[0]);
}

void test_override_limits(void) {
    btn.up = true;
    btn.clr = true; // OVERRIDE
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_LIFTING, logic.getCurrentState());

    // All motors are way past limit (15000 > 10000)
    positions[0] = 15000;
    positions[1] = 14800;
    positions[2] = 14800;
    positions[3] = 14800;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Motor 0 should STILL have throttle because of the override!
    TEST_ASSERT_NOT_EQUAL(0, throttles[0]);
}

void test_fault_deviation(void) {
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);

    // Motor 0 lags behind by more than MAX_DEVIATION (500)
    positions[0] = 0;
    positions[1] = 600;
    
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    
    // All throttles should be 0 in FAULT
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, throttles[i]);
    }
}

void test_fault_clear(void) {
    // Force into fault
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    positions[0] = 0;
    positions[1] = 600;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());

    // Release UP
    btn.up = false;
    
    // Press SET and CLR to clear fault
    btn.set = true;
    btn.clr = true;
    
    // Takes 5 seconds at 50Hz = 250 loops
    for (int i=0; i<249; i++) {
        logic.evaluate(btn, positions, throttles, fram_write_needed);
        TEST_ASSERT_EQUAL(SystemState::STATE_FAULT, logic.getCurrentState());
    }
    
    // The 250th loop should clear it
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_WAIT, logic.getCurrentState());
}

void test_set_down_zeros_positions(void) {
    // Setup some random positions
    positions[0] = 1234;
    positions[1] = 5678;
    
    // Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    TEST_ASSERT_EQUAL(SystemState::STATE_SET, logic.getCurrentState());

    // Press DOWN while holding SET
    btn.down = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Assert all positions became 0
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, positions[i]);
    }
}

void test_set_up_updates_upper_limit(void) {
    // Setup some random positions
    positions[0] = 1234;
    positions[1] = 5678;
    
    // Enter SET state
    btn.set = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Press UP while holding SET
    btn.up = true;
    logic.evaluate(btn, positions, throttles, fram_write_needed);
    
    // Upper limit should now be the max of all positions (5678)
    TEST_ASSERT_EQUAL(5678, logic.getUpperLimit());
}

void setup() {
    delay(2000); // Give time for serial monitor to connect
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

    UNITY_END();
}

void loop() {
    // Empty
}
