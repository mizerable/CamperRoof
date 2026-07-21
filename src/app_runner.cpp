#include "app_runner.h"
#include "core_logic.h"
#include "display.h"
#include "storage.h"
#include <Arduino.h>
#include <Wire.h>
#include "ble_controller.h"
#include "StallProtectingMotorSystem.h"

#include "pins.h"

// --- CONSTANTS ---
#define LOOP_PERIOD_MS 20     // 50Hz
#define DEBOUNCE_ITERATIONS 3 // 3 loops = 60ms

// --- GLOBALS ---
static SharedState systemState;
static SemaphoreHandle_t stateMutex;
static SemaphoreHandle_t i2cMutex;
static int32_t currentPositions[4] = {0, 0, 0, 0};
static int32_t upperLimit = 0;
static CoreLogic coreLogic;
static StallProtectingMotorSystem *g_motorSystem = nullptr;

// --- HELPER FUNCTIONS ---
static ButtonState get_debounced_buttons() {
  static ButtonState stable_state;
  static ButtonState last_read_state;
  static int stable_count = 0;

  ButtonState current_read;
  current_read.up = (digitalRead(PIN_BTN_UP) == LOW);
  current_read.down = (digitalRead(PIN_BTN_DOWN) == LOW);
  current_read.set = (digitalRead(PIN_BTN_SET) == LOW);
  current_read.clr = (digitalRead(PIN_BTN_CLEAR) == LOW);
  current_read.motor_sel = (digitalRead(PIN_BTN_MOTOR_SELECT) == LOW);

  if (current_read.up == last_read_state.up &&
      current_read.down == last_read_state.down &&
      current_read.set == last_read_state.set &&
      current_read.clr == last_read_state.clr &&
      current_read.motor_sel == last_read_state.motor_sel) {
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
static void MotorTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // 1. Read Inputs & Hardware/Mock Positions
    ButtonState btn = get_debounced_buttons();
    ButtonState ble_btn = BLEController::get_ble_buttons();
    
    // Logically OR the physical and BLE buttons
    btn.up = btn.up || ble_btn.up;
    btn.down = btn.down || ble_btn.down;
    btn.set = btn.set || ble_btn.set;
    btn.clr = btn.clr || ble_btn.clr;
    btn.motor_sel = btn.motor_sel || ble_btn.motor_sel;
    
    int32_t deltas[4] = {0, 0, 0, 0};
    g_motorSystem->getDeltas(deltas);

    for (int i = 0; i < 4; i++) {
        currentPositions[i] += deltas[i];
    }

    g_motorSystem->updateClearButton(btn.clr);

    // 2. Evaluate Core Logic
    int16_t throttles[4];

    coreLogic.evaluate(btn, currentPositions, throttles);

    // 3. Apply Hardware/Mock Outputs
    g_motorSystem->setThrottles(throttles);

    // 4. Persistence & UI Sync
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        write_state_to_fram(currentPositions, coreLogic.getUpperLimit());
        xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      systemState.state = coreLogic.getCurrentState();
      systemState.upperLimit = coreLogic.getUpperLimit();
      systemState.buttons = btn;
      for (int i = 0; i < 4; i++) {
        systemState.motors[i].currentPosition = currentPositions[i];
        systemState.motors[i].currentThrottle = throttles[i];
        systemState.motors[i].state = g_motorSystem->getMotorState(i);
      }
      xSemaphoreGive(stateMutex);
    }

    // 5. Wait for exact 50Hz timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// --- CORE 0: DISPLAY TASK ---
static void DisplayTask(void *pvParameters) {
  SharedState localCopy;
  for (;;) {
    // Safely copy shared state
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      localCopy = systemState;
      xSemaphoreGive(stateMutex);

      // Update physical screen
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          update_display(&localCopy);
          xSemaphoreGive(i2cMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz screen refresh
  }
}

void AppRunner::start(IMotorSystem *motorSystem) {
  g_motorSystem = new StallProtectingMotorSystem(motorSystem);

  Serial.begin(115200);

  // Initialize I2C Bus for both Display and FRAM
  Wire.begin();

  // Initialize buttons with pullups
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_SET, INPUT_PULLUP);
  pinMode(PIN_BTN_CLEAR, INPUT_PULLUP);
  pinMode(PIN_BTN_MOTOR_SELECT, INPUT_PULLUP);

  // Initialize Motor Subsystem (Hardware or Mock)
  g_motorSystem->init();

  // Initialize BLE Controller
  BLEController::init();

  // Initialize Other Subsystems
  setup_storage();
  setup_display();

  // Load state from physical I2C FRAM
  read_state_from_fram(currentPositions, &upperLimit);

  // Initialize Shared State
  coreLogic.setInitialState(currentPositions, upperLimit);
  systemState.state = coreLogic.getCurrentState();
  systemState.upperLimit = coreLogic.getUpperLimit();
  for (int i = 0; i < 4; i++) {
    systemState.motors[i].currentPosition = currentPositions[i];
    systemState.motors[i].currentThrottle = 0;
  }

  stateMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  // Create Tasks
  xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL,
                          2, // High priority
                          NULL,
                          1 // Core 1 (App Core)
  );

  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, NULL,
                          1, // Lower priority
                          NULL,
                          0 // Core 0 (Pro Core)
  );
}
