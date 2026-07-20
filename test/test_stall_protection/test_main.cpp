#include <Arduino.h>
#include <unity.h>
#include "StallProtectingMotorSystem.h"
#include "IMotorSystem.h"

// A simple mock for the underlying motor system
class MockMotor : public IMotorSystem {
public:
    int32_t nextDeltas[4];
    int16_t lastThrottles[4];

    MockMotor() {
        for(int i=0; i<4; i++) {
            nextDeltas[i] = 0;
            lastThrottles[i] = 0;
        }
    }

    void init() override {}

    void getDeltas(int32_t deltas[4]) override {
        for(int i=0; i<4; i++) {
            deltas[i] = nextDeltas[i];
        }
    }

    void setThrottles(const int16_t throttles[4]) override {
        for(int i=0; i<4; i++) {
            lastThrottles[i] = throttles[i];
        }
    }
};

MockMotor* mockMotor;
StallProtectingMotorSystem* stallSystem;

void setUp(void) {
    mockMotor = new MockMotor();
    stallSystem = new StallProtectingMotorSystem(mockMotor);
    stallSystem->init();
}

void tearDown(void) {
    delete stallSystem;
    delete mockMotor;
}

void test_initial_state_is_ok(void) {
    for (int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(i));
    }
}

void test_no_stall_on_steady_movement(void) {
    int16_t reqThrottles[4] = {50, 50, 50, 50};
    
    // Simulate 20 ticks of steady movement
    for (int t=0; t<20; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 20;
        
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
        
        for(int i=0; i<4; i++) {
            TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(i));
            TEST_ASSERT_EQUAL(50, mockMotor->lastThrottles[i]);
        }
    }
}

void test_no_stall_on_minor_bog_down_without_throttle_increase(void) {
    int16_t reqThrottles[4] = {50, 50, 50, 50};
    
    // 10 ticks of normal speed
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 20;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
    // Now bog down to delta=10, BUT throttle stays at 50
    bool stalled = false;
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 10;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
        if (stallSystem->getMotorState(0) != MotorState::OK) {
            stalled = true;
            break;
        }
    }
    
    // Should NOT stall because throttle didn't ramp up by 20%
    TEST_ASSERT_FALSE(stalled);
}

void test_stall_triggers_on_throttle_ramp_with_bog_down(void) {
    int16_t reqThrottles[4] = {50, 50, 50, 50};
    
    // 10 ticks of normal speed
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 20;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
    // Now bog down to delta=5, AND throttle ramps up to 75 (increase > 20)
    for(int i=0; i<4; i++) reqThrottles[i] = 75;
    
    bool stalled = false;
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 5;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
        if (stallSystem->getMotorState(0) == MotorState::STALLED_LIFTING) {
            stalled = true;
            break;
        }
    }
    
    // Should stall because throttle ramped up by > 20 but speed dropped
    TEST_ASSERT_TRUE(stalled);
    TEST_ASSERT_EQUAL(0, mockMotor->lastThrottles[0]);
}

void test_stall_triggers_when_throttle_maxed_out(void) {
    int16_t reqThrottles[4] = {100, 100, 100, 100};
    
    // 10 ticks of normal speed at max throttle
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 40;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
    // Now speed drops to 0 (dead stop), throttle is still 100
    // It can't increase by 20, but it is >= 95
    bool stalled = false;
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 0;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
        if (stallSystem->getMotorState(0) == MotorState::STALLED_LIFTING) {
            stalled = true;
            break;
        }
    }
    
    TEST_ASSERT_TRUE(stalled);
    TEST_ASSERT_EQUAL(0, mockMotor->lastThrottles[0]);
}

void test_clear_button_overrides_stall(void) {
    // Cause a stall
    test_stall_triggers_when_throttle_maxed_out();
    
    // Now hold CLEAR
    int16_t reqThrottles[4] = {100, 100, 100, 100};
    int32_t deltas[4];
    stallSystem->getDeltas(deltas);
    stallSystem->updateClearButton(true); // CLEAR HELD!
    stallSystem->setThrottles(reqThrottles);
    
    TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(0));
    TEST_ASSERT_EQUAL(100, mockMotor->lastThrottles[0]); // Throttle passes through!
}

void test_opposite_direction_clears_stall(void) {
    // Cause a lifting stall
    test_stall_triggers_when_throttle_maxed_out();
    TEST_ASSERT_EQUAL(MotorState::STALLED_LIFTING, stallSystem->getMotorState(0));
    
    // Command LOWERING
    int16_t reqThrottles[4] = {-50, -50, -50, -50};
    int32_t deltas[4];
    stallSystem->getDeltas(deltas);
    stallSystem->updateClearButton(false);
    stallSystem->setThrottles(reqThrottles);
    
    TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(0));
    TEST_ASSERT_EQUAL(-50, mockMotor->lastThrottles[0]); // Passed through
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_ok);
    RUN_TEST(test_no_stall_on_steady_movement);
    RUN_TEST(test_no_stall_on_minor_bog_down_without_throttle_increase);
    RUN_TEST(test_stall_triggers_on_throttle_ramp_with_bog_down);
    RUN_TEST(test_stall_triggers_when_throttle_maxed_out);
    RUN_TEST(test_clear_button_overrides_stall);
    RUN_TEST(test_opposite_direction_clears_stall);
    UNITY_END();
}

void loop() {}
