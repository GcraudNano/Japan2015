#include "GCSwitch.h"

GCSwitch::GCSwitch(uint8_t _pin, uint8_t _onState) : pin(_pin), onState(_onState)
{
    
}

GCSwitch::operator bool()
{
    return pin == onState;
}
