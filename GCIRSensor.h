#ifndef __GCIRSensor__
#define __GCIRSensor__

#include "GCRobot.h"
#include "GCSharedObjects.h"

class GCIRSensor {
    static const int farBase;
    static const int middleBase;
    static const int timeOut;
    DigitalIn pin;
    int       deg1;
    int       deg2;
    static int median(int *arr, int count);
    
public:
    static GCraudNano::GCBallDistance ballDistance;
    int value;
    
    GCIRSensor(uint8_t pin, GCraudNano::GCIRSensorDegrees deg1, GCraudNano::GCIRSensorDegrees deg2);
    static void getValues(GCIRSensor *sensors, int count);
    static GCraudNano::GCBallDegrees getMedian(GCIRSensor *sensors, int count);
};

#endif
