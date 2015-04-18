#include "GCLineSensor.h"
using GCraudNano::GCLineDegrees;

#undef degrees
GCLineSensor::GCLineSensor(uint8_t _pin, int _whiteValue, GCLineDegrees _degrees) : pin(_pin), whiteValue(_whiteValue), degrees(_degrees)
{
    
}

GCLineDegrees GCLineSensor::isWhite()
{
    return pin >= whiteValue ? degrees : 0;
}

int GCLineSensor::getValue()
{
    return pin;
}
