#include "bus.h"
#include <Arduino.h>

// Constructors
Bus::Bus(uint8_t ctl_pin, uint8_t cs_pin, float max_current_draw)
    : ctl_pin(ctl_pin), cs_pin(cs_pin), max_current_draw(max_current_draw), enabled(false), toggleable(true) {
}

Bus::Bus(unit8_t cs_pin, float max_current_draw)
    : cs_pin(cs_pin), max_current_draw(max_current_draw), toggleable(false) {
}

// Enable the bus
void Bus::enable() {
    if (toggleable) {
        digitalWrite(ctl_pin, HIGH);
        enabled = true;
    }
    
}

// Disable the bus
void Bus::disable() {
    if (toggleable) {
        digitalWrite(ctl_pin, LOW);
        enabled = false;
    }
}

// Read current from the bus in amps
float Bus::readCurrent() {
    int rawValue = analogRead(cs_pin);
    // Conversion from analog value to amps (assuming linear conversion)
    float current = static_cast<float>(rawValue) / 1023.0 * 5.0; // Assuming 5V reference
    return current;
}

// Check for overcurrent
bool Bus::overcurrent() {
    float current = readCurrent();
    return current > max_current_draw;
}

bool Bus::enabled() {
    return enabled;
}
void Bus::init() {
    if (togglable) {
        pinMode(ctl_pin, OUTPUT)
    };
}