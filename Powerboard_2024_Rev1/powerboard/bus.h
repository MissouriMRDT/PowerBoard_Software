#ifndef bus_h
#define bus_h

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
    uint8_t ctl_pin;
    uint8_t cs_pin;
    float max_current_draw;
    bool enabled;
    bool toggleable;
};

#endif