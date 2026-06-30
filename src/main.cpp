#include <Arduino.h>
#include <Wire.h>
#include "pcnt_setup.h"
#include "motor_control.h"
#include "storage.h"
#include "display.h"
#include "core_logic.h"

#ifndef PIO_UNIT_TESTING

// --- PINS ---
#define PIN_BTN_UP 13
#define PIN_BTN_DOWN 14
#define PIN_BTN_SET 27
#define PIN_BTN_CLEAR 33

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// --- CONSTANTS ---
#define MAX_DEVIATION 500 // Maximum allowed pulse difference between any two motors
#define BASE_THROTTLE 50
#define MIN_THROTTLE 20
#define MAX_THROTTLE 80
#define LOOP_PERIOD_MS 20 // 50Hz
#define DEBOUNCE_ITERATIONS 3 // 3 loops = 60ms

// --- GLOBALS ---
SharedState systemState;
SemaphoreHandle_t stateMutex;
int32_t currentPositions[4] = {0, 0, 0, 0};
int32_t upperLimit = 0;
CoreLogic coreLogic;

// Button state tracking is defined in system_types.h

// --- TASKS ---
void MotorTask(void *pvParameters);
void DisplayTask(void *pvParameters);

void setup() {
    Serial.begin(115200);

    // Initialize I2C Bus for both Display and FRAM
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // Initialize buttons with pullups
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_SET, INPUT_PULLUP);
    pinMode(PIN_BTN_CLEAR, INPUT_PULLUP);

    // Initialize Subsystems
    setup_pcnt();
    setup_motors();
    setup_storage();
    setup_display();

    // Read saved state from FRAM
    read_state_from_fram(currentPositions, &upperLimit);

    // Initialize Shared State
    coreLogic.setInitialState(currentPositions, upperLimit);
    systemState.state = coreLogic.getCurrentState();
    systemState.upperLimit = coreLogic.getUpperLimit();
    for(int i=0; i<4; i++) {
        systemState.motors[i].currentPosition = currentPositions[i];
        systemState.motors[i].currentThrottle = 0;
    }

    stateMutex = xSemaphoreCreateMutex();

    // Create Tasks
    xTaskCreatePinnedToCore(
        MotorTask,
        "MotorTask",
        4096,
        NULL,
        2, // High priority
        NULL,
        1  // Core 1 (App Core)
    );

    xTaskCreatePinnedToCore(
        DisplayTask,
        "DisplayTask",
        4096,
        NULL,
        1, // Lower priority
        NULL,
        0  // Core 0 (Pro Core)
    );
}

void loop() {
    // Empty, FreeRTOS handles the tasks
    vTaskDelete(NULL);
}

// --- HELPER FUNCTIONS ---

ButtonState get_debounced_buttons() {
    static ButtonState stable_state;
    static ButtonState last_read_state;
    static int stable_count = 0;

    ButtonState current_read;
    current_read.up = (digitalRead(PIN_BTN_UP) == LOW);
    current_read.down = (digitalRead(PIN_BTN_DOWN) == LOW);
    current_read.set = (digitalRead(PIN_BTN_SET) == LOW);
    current_read.clr = (digitalRead(PIN_BTN_CLEAR) == LOW);

    if (current_read.up == last_read_state.up &&
        current_read.down == last_read_state.down &&
        current_read.set == last_read_state.set &&
        current_read.clr == last_read_state.clr) {
        stable_count++;
        if (stable_count >= DEBOUNCE_ITERATIONS) {
            stable_state = current_read;
            stable_count = DEBOUNCE_ITERATIONS; // Prevent overflow
        }
    } else {
        stable_count = 0;
        last_read_state = current_read;
    }

    return stable_state;
}

// --- CORE 1: MOTOR TASK ---
void MotorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1. Read Inputs & PCNT
        ButtonState btn = get_debounced_buttons();
        update_pcnt_counts(currentPositions);

        // 2. Evaluate Core Logic
        int16_t throttles[4];
        bool fram_write_needed = false;
        
        coreLogic.evaluate(btn, currentPositions, throttles, fram_write_needed);

        // 3. Apply Hardware Outputs
        for (int i = 0; i < 4; i++) {
            set_motor_throttle(i, throttles[i]);
        }

        // 4. Persistence & UI Sync
        if (fram_write_needed) {
            write_state_to_fram(currentPositions, coreLogic.getUpperLimit());
        }

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            systemState.state = coreLogic.getCurrentState();
            systemState.upperLimit = coreLogic.getUpperLimit();
            for(int i=0; i<4; i++) {
                systemState.motors[i].currentPosition = currentPositions[i];
                systemState.motors[i].currentThrottle = throttles[i];
            }
            xSemaphoreGive(stateMutex);
        }

        // 5. Wait for exact 50Hz timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// --- CORE 0: DISPLAY TASK ---
void DisplayTask(void *pvParameters) {
    SharedState localCopy;
    for (;;) {
        // Safely copy shared state
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            localCopy = systemState;
            xSemaphoreGive(stateMutex);
            
            // Update physical screen
            update_display(&localCopy);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz screen refresh
    }
}

#endif // PIO_UNIT_TESTING