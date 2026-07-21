#pragma once

#include <Arduino.h>

// ==========================================
// CAMPER ROOF HARDWARE PIN CONFIGURATION
// ==========================================

// --- BUTTONS ---
#define PIN_BTN_UP    A12
#define PIN_BTN_DOWN  A6
#define PIN_BTN_SET   A10
#define PIN_BTN_CLEAR A9
#define PIN_BTN_MOTOR_SELECT RX // Moved from A2 to avoid input-only pin

// --- CYTRON MOTOR CONTROLLERS (SIMPLIFIED SERIAL) ---
#define PIN_CYTRON1_TX A5
#define PIN_CYTRON2_TX SCK

// --- PCNT (PULSE COUNTERS / ENCODERS) ---
// Each motor has 2 quadrature pins (Phase A and Phase B)
#define PIN_M1_PHASE_A A0
#define PIN_M1_PHASE_B A1

#define PIN_M2_PHASE_A MOSI
#define PIN_M2_PHASE_B MISO

#define PIN_M3_PHASE_A A8
#define PIN_M3_PHASE_B A7

#define PIN_M4_PHASE_A TX
#define PIN_M4_PHASE_B A11

// --- I2C BUS (DISPLAY & FRAM) ---
// Uses default SDA and SCL for the board unless overridden here
#define PIN_I2C_SDA SDA
#define PIN_I2C_SCL SCL

// --- ADAFRUIT ESP32 V2 NEOPIXEL ---
#define PIN_NEOPIXEL 0
#define PIN_NEOPIXEL_POWER 2
