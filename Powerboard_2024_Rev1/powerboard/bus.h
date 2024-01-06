#ifndef bus_h
#define bus_h
#include "RoveComm.h"

class Bus {
public:
    // Constructor
    Bus(uint8_t ctl_pin, uint8_t cs_pin, float max_current_draw);

    // Member functions
    void enable();
    void disable();
    float readCurrent();
    bool overcurrent();

private:
    // Member variables
    uint8_t ctl_pin;
    uint8_t cs_pin;
    float max_current_draw;
    bool enabled;
};

#endif