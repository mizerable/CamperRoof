#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "pins.h"
#include "status_led.h"

// 1 NeoPixel on PIN_NEOPIXEL
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

static uint32_t last_update = 0;
static uint16_t hue = 0;

void setup_status_led() {
    // Power on the NeoPixel logic
    pinMode(PIN_NEOPIXEL_POWER, OUTPUT);
    digitalWrite(PIN_NEOPIXEL_POWER, HIGH);

    pixel.begin();
    pixel.setBrightness(50); // vibrant but not blinding
    pixel.show();
}

void update_status_led() {
    // Non-blocking animation: update color every 20ms
    uint32_t now = millis();
    if (now - last_update >= 20) {
        last_update = now;
        
        // Cycle hue continuously
        hue += 256; // 65536 / 256 = 256 steps
        
        // Convert HSV to RGB
        uint32_t rgbcolor = pixel.ColorHSV(hue, 255, 255);
        pixel.setPixelColor(0, rgbcolor);
        pixel.show();
    }
}
