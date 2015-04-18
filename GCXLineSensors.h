#ifndef __GCXLineSensors__
#define __GCXLineSensors__

#include "GCRobot.h"
#include "GCSharedObjects.h"

class GCXLineSensors {
public:
    enum DelegateOperation {
        isRightFar  = 1 << 0,
        isLeftFar   = 1 << 1,
        isBackFar   = 1 << 2,
        isBackClose = 1 << 3,
    };
    
private:
    GCraudNano::GCLineDegrees (*rightFrontLineSensorValue)();
    GCraudNano::GCLineDegrees (*rightBackLineSensorValue)();
    GCraudNano::GCLineDegrees (*leftFrontLineSensorValue)();
    GCraudNano::GCLineDegrees (*leftBackLineSensorValue)();
    bool (*sonicDelegate)(DelegateOperation operation);
    
public:
    GCXLineSensors();
    void setRightFrontLineSensorDelegate(GCraudNano::GCLineDegrees (*func)());
    void setRightBackLineSensorDelegate(GCraudNano::GCLineDegrees (*func)());
    void setLeftFrontLineSensorDelegate(GCraudNano::GCLineDegrees (*func)());
    void setLeftBackLineSensorDelegate(GCraudNano::GCLineDegrees (*func)());
    void setSonicDelegate(bool (*func)(DelegateOperation operation));
    GCraudNano::GCLineDegrees lineDegrees();
    bool __attribute__((deprecated)) shouldStop(GCraudNano::GCMoveDegrees direction);
    
};

#endif
