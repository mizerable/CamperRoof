#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ==========================================
// CAMPER ROOF HARDWARE PIN CONFIGURATION
// ==========================================

// --- BUTTONS ---
#define PIN_BTN_UP    A12
#define PIN_BTN_DOWN  A6
#define PIN_BTN_SET   A10
#define PIN_BTN_CLEAR A9

// --- CYTRON MOTOR CONTROLLERS (SIMPLIFIED SERIAL) ---
// Note: We only need TX pins to send commands to the Cytrons.
#define PIN_CYTRON1_TX TX   // Typically GPIO 17 on Feather ESP32
#define PIN_CYTRON2_TX RX   // Typically GPIO 16 on Feather ESP32

// --- PCNT (PULSE COUNTERS / ENCODERS) ---
// Each motor has 2 quadrature pins (Phase A and Phase B)
#define PIN_M1_PHASE_A A2
#define PIN_M1_PHASE_B A13

#define PIN_M2_PHASE_A A4
#define PIN_M2_PHASE_B A3

#define PIN_M3_PHASE_A A5
#define PIN_M3_PHASE_B MOSI

#define PIN_M4_PHASE_A MISO
#define PIN_M4_PHASE_B SDA

// --- I2C BUS (DISPLAY & FRAM) ---
// Uses default SDA and SCL for the board unless overridden here
#define PIN_I2C_SDA SDA
#define PIN_I2C_SCL SCL

#endif // PINS_H
