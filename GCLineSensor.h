#ifndef __GCLineSensor__
#define __GCLineSensor__

#include "GCRobot.h"
#include "GCSharedObjects.h"

class GCLineSensor {
    AnalogIn pin;
    int whiteValue;
    GCraudNano::GCLineDegrees degrees;
    
public:
    GCLineSensor(uint8_t pin, int whiteValue, GCraudNano::GCLineDegrees degrees);
    GCraudNano::GCLineDegrees isWhite();
    int getValue();
    
};

#endif
