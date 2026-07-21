#include <Arduino.h>
#include <unity.h>
#include "pins.h"
#include <vector>
#include <algorithm>

void setUp(void) {}
void tearDown(void) {}

void test_no_pin_collisions(void) {
    // Collect all defined pins in the system
    std::vector<int> all_pins = {
        PIN_BTN_UP,
        PIN_BTN_DOWN,
        PIN_BTN_SET,
        PIN_BTN_CLEAR,
        PIN_BTN_MOTOR_SELECT,
        
        PIN_CYTRON1_TX,
        PIN_CYTRON2_TX,
        
        PIN_M1_PHASE_A,
        PIN_M1_PHASE_B,
        
        PIN_M2_PHASE_A,
        PIN_M2_PHASE_B,
        
        PIN_M3_PHASE_A,
        PIN_M3_PHASE_B,
        
        PIN_M4_PHASE_A,
        PIN_M4_PHASE_B,
        
        PIN_I2C_SDA,
        PIN_I2C_SCL
    };

    // We must ensure there are no duplicate pin numbers assigned
    for (size_t i = 0; i < all_pins.size(); i++) {
        for (size_t j = i + 1; j < all_pins.size(); j++) {
            if (all_pins[i] == all_pins[j]) {
                char error_msg[100];
                snprintf(error_msg, sizeof(error_msg), "Pin collision detected! Pin %d is used multiple times.", all_pins[i]);
                TEST_FAIL_MESSAGE(error_msg);
            }
        }
    }
}

#ifndef PIO_UNIT_TESTING_MOCK
void setup() {
    delay(5000);
    UNITY_BEGIN();
    RUN_TEST(test_no_pin_collisions);
    UNITY_END();
}

void loop() {}
#else
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_no_pin_collisions);
    return UNITY_END();
}
#endif
