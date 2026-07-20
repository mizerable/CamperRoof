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
    
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 20;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
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
    
    TEST_ASSERT_FALSE(stalled);
}

void test_stall_triggers_on_throttle_ramp_with_bog_down(void) {
    int16_t reqThrottles[4] = {50, 50, 50, 50};
    
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 20;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
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
    
    TEST_ASSERT_TRUE(stalled);
    TEST_ASSERT_EQUAL(0, mockMotor->lastThrottles[0]);
}

void test_stall_triggers_when_throttle_maxed_out(void) {
    int16_t reqThrottles[4] = {100, 100, 100, 100};
    
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 40;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
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

void test_group_halt_lifting(void) {
    int16_t reqThrottles[4] = {100, 100, 100, 100};
    
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = 40;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
    // Only motor 0 stalls, the others are still trying to move
    bool stalled = false;
    for (int t=0; t<10; t++) {
        mockMotor->nextDeltas[0] = 0; // Motor 0 dead stop
        mockMotor->nextDeltas[1] = 40;
        mockMotor->nextDeltas[2] = 40;
        mockMotor->nextDeltas[3] = 40;
        
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
    // Because motor 0 stalled while lifting, ALL motors should be forced to 0 throttle
    for(int i=0; i<4; i++) {
        TEST_ASSERT_EQUAL(0, mockMotor->lastThrottles[i]);
    }
}

void test_no_group_halt_lowering(void) {
    int16_t reqThrottles[4] = {-100, -100, -100, -100};
    
    for (int t=0; t<10; t++) {
        for(int i=0; i<4; i++) mockMotor->nextDeltas[i] = -40;
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
    }
    
    // Motor 0 hits the bottom and stalls
    bool stalled = false;
    for (int t=0; t<10; t++) {
        mockMotor->nextDeltas[0] = 0; // Motor 0 seated
        mockMotor->nextDeltas[1] = -40;
        mockMotor->nextDeltas[2] = -40;
        mockMotor->nextDeltas[3] = -40;
        
        int32_t deltas[4];
        stallSystem->getDeltas(deltas);
        stallSystem->updateClearButton(false);
        stallSystem->setThrottles(reqThrottles);
        if (stallSystem->getMotorState(0) == MotorState::STALLED_LOWERING) {
            stalled = true;
            break;
        }
    }
    
    TEST_ASSERT_TRUE(stalled);
    // Motor 0 should be 0 throttle
    TEST_ASSERT_EQUAL(0, mockMotor->lastThrottles[0]);
    // Motors 1, 2, 3 should STILL be commanding -100 to seat themselves!
    for(int i=1; i<4; i++) {
        TEST_ASSERT_EQUAL(-100, mockMotor->lastThrottles[i]);
    }
}

void test_clear_button_overrides_stall(void) {
    test_stall_triggers_when_throttle_maxed_out();
    
    int16_t reqThrottles[4] = {100, 100, 100, 100};
    int32_t deltas[4];
    stallSystem->getDeltas(deltas);
    stallSystem->updateClearButton(true); 
    stallSystem->setThrottles(reqThrottles);
    
    TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(0));
    TEST_ASSERT_EQUAL(100, mockMotor->lastThrottles[0]); 
}

void test_opposite_direction_clears_stall(void) {
    test_stall_triggers_when_throttle_maxed_out();
    TEST_ASSERT_EQUAL(MotorState::STALLED_LIFTING, stallSystem->getMotorState(0));
    
    int16_t reqThrottles[4] = {-50, -50, -50, -50};
    int32_t deltas[4];
    stallSystem->getDeltas(deltas);
    stallSystem->updateClearButton(false);
    stallSystem->setThrottles(reqThrottles);
    
    TEST_ASSERT_EQUAL(MotorState::OK, stallSystem->getMotorState(0));
    TEST_ASSERT_EQUAL(-50, mockMotor->lastThrottles[0]); 
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_ok);
    RUN_TEST(test_no_stall_on_steady_movement);
    RUN_TEST(test_no_stall_on_minor_bog_down_without_throttle_increase);
    RUN_TEST(test_stall_triggers_on_throttle_ramp_with_bog_down);
    RUN_TEST(test_stall_triggers_when_throttle_maxed_out);
    RUN_TEST(test_group_halt_lifting);
    RUN_TEST(test_no_group_halt_lowering);
    RUN_TEST(test_clear_button_overrides_stall);
    RUN_TEST(test_opposite_direction_clears_stall);
    UNITY_END();
}

void loop() {}
