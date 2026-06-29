#include <Arduino.h>
#include <Wire.h>
#include "pcnt_setup.h"
#include "motor_control.h"
#include "storage.h"
#include "display.h"

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

// Button state tracking
struct ButtonState {
    bool up = false;
    bool down = false;
    bool set = false;
    bool clr = false;
};

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
    systemState.state = SystemState::STATE_WAIT;
    systemState.upperLimit = upperLimit;
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

// Float midpoint logic
void apply_proportional_throttle(bool isLifting) {
    int32_t min_pos = currentPositions[0];
    int32_t max_pos = currentPositions[0];

    for (int i = 1; i < 4; i++) {
        if (currentPositions[i] < min_pos) min_pos = currentPositions[i];
        if (currentPositions[i] > max_pos) max_pos = currentPositions[i];
    }

    int32_t midpoint = (min_pos + max_pos) / 2;

    for (int i = 0; i < 4; i++) {
        int throttle = BASE_THROTTLE;
        int32_t deviation = currentPositions[i] - midpoint;

        // If lifting, a motor that is higher than midpoint needs less throttle.
        // If lowering, a motor that is higher than midpoint needs more throttle.
        if (isLifting) {
            // Mapping deviation [-MAX/2, MAX/2] to [MAX_THROTTLE, MIN_THROTTLE]
            // deviation > 0 means it's ahead, so reduce throttle.
            throttle = map(deviation, -(MAX_DEVIATION/2), (MAX_DEVIATION/2), MAX_THROTTLE, MIN_THROTTLE);
        } else {
            // If lowering, deviation > 0 means it's higher (falling behind the lowering process), so increase throttle.
            throttle = map(deviation, -(MAX_DEVIATION/2), (MAX_DEVIATION/2), MIN_THROTTLE, MAX_THROTTLE);
        }

        // Clamp
        if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;
        if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;

        // Direction mapping
        if (!isLifting) {
            throttle = -throttle;
        }

        // Check Individual limits
        if (isLifting && currentPositions[i] >= upperLimit) {
            throttle = 0;
        }
        if (!isLifting && currentPositions[i] <= 0) {
            throttle = 0;
        }

        set_motor_throttle(i, throttle);
        
        // Update shared state
        if (xSemaphoreTake(stateMutex, 0) == pdTRUE) {
            systemState.motors[i].currentThrottle = throttle;
            xSemaphoreGive(stateMutex);
        }
    }
}

// --- CORE 1: MOTOR TASK ---
void MotorTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    SystemState currentState = SystemState::STATE_WAIT;
    int fault_clear_timer = 0;

    for (;;) {
        // 1. Read Inputs & PCNT
        ButtonState btn = get_debounced_buttons();
        update_pcnt_counts(currentPositions);

        // 2. Check Fault Condition (Max Deviation)
        int32_t min_pos = currentPositions[0];
        int32_t max_pos = currentPositions[0];
        for (int i = 1; i < 4; i++) {
            if (currentPositions[i] < min_pos) min_pos = currentPositions[i];
            if (currentPositions[i] > max_pos) max_pos = currentPositions[i];
        }
        
        if ((currentState == SystemState::STATE_LIFTING || currentState == SystemState::STATE_LOWERING) 
            && (max_pos - min_pos > MAX_DEVIATION)) {
            currentState = SystemState::STATE_FAULT;
            stop_all_motors();
        }

        // 3. State Machine
        switch (currentState) {
            case SystemState::STATE_WAIT:
                stop_all_motors();
                if (btn.up && !btn.down && !btn.set && !btn.clr) {
                    currentState = SystemState::STATE_LIFTING;
                } else if (!btn.up && btn.down && !btn.set && !btn.clr) {
                    currentState = SystemState::STATE_LOWERING;
                } else if (!btn.up && !btn.down && btn.set && !btn.clr) {
                    currentState = SystemState::STATE_SET;
                }
                break;

            case SystemState::STATE_LIFTING:
                if (!btn.up) {
                    currentState = SystemState::STATE_WAIT;
                } else {
                    apply_proportional_throttle(true);
                }
                break;

            case SystemState::STATE_LOWERING:
                if (!btn.down) {
                    currentState = SystemState::STATE_WAIT;
                } else {
                    apply_proportional_throttle(false);
                }
                break;

            case SystemState::STATE_FAULT:
                stop_all_motors();
                if (btn.set && btn.clr) {
                    fault_clear_timer++;
                    if (fault_clear_timer >= (5000 / LOOP_PERIOD_MS)) { // 5 seconds
                        currentState = SystemState::STATE_WAIT;
                        fault_clear_timer = 0;
                    }
                } else {
                    fault_clear_timer = 0;
                }
                break;

            case SystemState::STATE_SET:
                stop_all_motors();
                if (!btn.set) {
                    currentState = SystemState::STATE_WAIT;
                } else {
                    if (btn.down) {
                        for (int i = 0; i < 4; i++) currentPositions[i] = 0;
                        write_state_to_fram(currentPositions, upperLimit);
                    }
                    if (btn.up) {
                        upperLimit = max_pos;
                        write_state_to_fram(currentPositions, upperLimit);
                    }
                }
                break;
        }

        // 4. Persistence & UI Sync
        write_state_to_fram(currentPositions, upperLimit);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            systemState.state = currentState;
            systemState.upperLimit = upperLimit;
            for(int i=0; i<4; i++) {
                systemState.motors[i].currentPosition = currentPositions[i];
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