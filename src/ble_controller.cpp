#include "ble_controller.h"
#include <NimBLEDevice.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static ButtonState current_ble_buttons = {false, false, false, false, false};
static bool device_connected = false;

class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        device_connected = true;
    }
    void onDisconnect(NimBLEServer* pServer) {
        device_connected = false;
        // Reset all buttons if disconnected
        current_ble_buttons = {false, false, false, false, false};
        // Restart advertising
        NimBLEDevice::startAdvertising();
    }
};

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        BLEController::simulate_ble_rx(rxValue.c_str());
    }
};

void BLEController::simulate_ble_rx(const char* data) {
    if (data == nullptr) return;
    std::string s(data);
    size_t pos = 0;
    // Look for Adafruit Control Pad packet signature "!B"
    while ((pos = s.find("!B", pos)) != std::string::npos) {
        if (pos + 3 < s.length()) { // Need at least ! B [btn] [state]
            char btn = s[pos + 2];
            bool pressed = (s[pos + 3] == '1');
            switch (btn) {
                case '5': current_ble_buttons.up = pressed; break;
                case '6': current_ble_buttons.down = pressed; break;
                case '2': current_ble_buttons.motor_sel = pressed; break;
                case '1': current_ble_buttons.set = pressed; break;
                case '3': current_ble_buttons.clr = pressed; break;
            }
            pos += 4; // Advance past this packet
        } else {
            break; // Incomplete packet
        }
    }
}

void BLEController::init() {
    NimBLEDevice::init("CamperRoof");
    
    // Set power to maximum for better range
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    
    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);

    NimBLECharacteristic *pTxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,
                                         NIMBLE_PROPERTY::NOTIFY
                                       );
                                       
    (void)pTxCharacteristic; // Not used yet, but standard for UART

    NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
                                       );

    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
}

ButtonState BLEController::get_ble_buttons() {
    return current_ble_buttons;
}

bool BLEController::is_connected() {
    return device_connected;
}
