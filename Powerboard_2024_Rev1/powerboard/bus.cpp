#include "bus.h"
#include <Arduino.h>

// Constructors

Bus::Bus(uint8_t ctl_pin, uint8_t cs_pin, float max_current_draw)
    : m_ctl_pin(ctl_pin), m_cs_pin(cs_pin), m_max_current_draw(max_current_draw), m_enabled(false), m_toggleable(true) {
}

Bus::Bus(uint8_t cs_pin, float max_current_draw)
    : m_cs_pin(cs_pin), m_max_current_draw(max_current_draw), m_enabled(true), m_toggleable(false) {
}

// Enable the bus
void Bus::enable() {
    if (m_toggleable) {
        digitalWrite(m_ctl_pin, HIGH);
        m_enabled = true;
    }
}

// Disable the bus
void Bus::disable() {
    if (m_toggleable) {
        digitalWrite(m_ctl_pin, LOW);
        m_enabled = false;
    }
}

// Read current from the bus in amps
float Bus::readCurrent() {
    int rawValue = analogRead(m_cs_pin);
    // Conversion from analog value to amps (assuming linear conversion)
    float current = rawValue * 5.0 / 1023.0; // Assuming 5V reference
    return current;
}

// Check for overcurrent
bool Bus::overcurrent() {
    float current = readCurrent();
    return current > m_max_current_draw;
}

bool Bus::enabled() {
    return m_enabled;
}
void Bus::init() {
    if (m_toggleable) {
        pinMode(m_ctl_pin, OUTPUT);
    }
}