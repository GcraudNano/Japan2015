#ifndef __GCSonicSensor__
#define __GCSonicSensor__

#include "GCRobot.h"

class GCSonicSensor {
    static const int timeout;
    static const int interval;
    DigitalInOut pin;
    unsigned long closeToWall;
    unsigned long farFromWall;
    uint8_t rawPin;
    
public:
    GCSonicSensor(uint8_t pin, unsigned long closeToWall);
    GCSonicSensor(uint8_t pin, unsigned long closeToWall, unsigned long farFromWall);
    bool isClose();
    bool isFar();
    unsigned long getValue();
    unsigned long value;
};

#endif
