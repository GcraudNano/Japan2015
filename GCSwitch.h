#ifndef __GCSwitch__
#define __GCSwitch__

#include "GCRobot.h"

class GCSwitch {
    DigitalIn pin;
    uint8_t   onState;
    
public:
    GCSwitch(uint8_t pin, uint8_t onState);
    operator bool();
};

#endif
