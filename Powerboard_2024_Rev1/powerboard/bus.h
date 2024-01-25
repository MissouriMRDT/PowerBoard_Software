#ifndef bus_h
#define bus_h

#include <stdint.h>

class Bus {
public:
    // Constructor
    Bus(uint8_t ctl_pin, uint8_t cs_pin, float max_current_draw);
    Bus(uint8_t cs_pin, float max_current_draw);

    // Member functions
    void enable();
    void disable();
    void init();
    float readCurrent();
    bool overcurrent();
    bool enabled();

private:
    // Member variables
    uint8_t m_ctl_pin;
    uint8_t m_cs_pin;
    float m_max_current_draw;
    bool m_enabled;
    bool m_toggleable;
};

#endif