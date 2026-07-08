#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

#include <Arduino.h>
#include "system_types.h"

class BLEController {
public:
    static void init();
    static ButtonState get_ble_buttons();
    static bool is_connected();
    static void simulate_ble_rx(const char* data);
};

#endif // BLE_CONTROLLER_H
