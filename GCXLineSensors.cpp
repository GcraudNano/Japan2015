#include "GCXLineSensors.h"
using namespace GCraudNano;

GCXLineSensors::GCXLineSensors() : rightFrontLineSensorValue(NULL), rightBackLineSensorValue(NULL), leftFrontLineSensorValue(NULL), leftBackLineSensorValue(NULL), sonicDelegate(NULL)
{
    
}
void GCXLineSensors::setRightFrontLineSensorDelegate(GCraudNano::GCLineDegrees (*func)())
{
    rightFrontLineSensorValue = func;
}

void GCXLineSensors::setRightBackLineSensorDelegate(GCraudNano::GCLineDegrees (*func)())
{
    rightBackLineSensorValue = func;
}

void GCXLineSensors::setLeftFrontLineSensorDelegate(GCraudNano::GCLineDegrees (*func)())
{
    leftFrontLineSensorValue = func;
}

void GCXLineSensors::setLeftBackLineSensorDelegate(GCraudNano::GCLineDegrees (*func)())
{
    leftBackLineSensorValue = func;
}

void GCXLineSensors::setSonicDelegate(bool (*func)(GCXLineSensors::DelegateOperation))
{
    sonicDelegate = func;
}

GCLineDegrees GCXLineSensors::lineDegrees()
{
    return rightFrontLineSensorValue() | rightBackLineSensorValue() | leftFrontLineSensorValue() | leftBackLineSensorValue();
}

bool GCXLineSensors::shouldStop(GCMoveDegrees direction)
{
    switch (direction) {
        case GCMoveDegrees0:
            return !sonicDelegate(isBackClose) && rightFrontLineSensorValue() && leftFrontLineSensorValue();
            break;
            
        case GCMoveDegrees45:
            return !sonicDelegate(static_cast<DelegateOperation>(isBackClose | isLeftFar)) && leftFrontLineSensorValue();
            break;
            
        case GCMoveDegrees_45:
            return !sonicDelegate(static_cast<DelegateOperation>(isBackClose | isRightFar)) && rightFrontLineSensorValue();
            break;
            
        case GCMoveDegrees90:
            return !sonicDelegate(isLeftFar) && (leftFrontLineSensorValue() || leftBackLineSensorValue());
            break;
            
        case GCMoveDegrees_90:
            return !sonicDelegate(isRightFar) && (rightFrontLineSensorValue() || rightBackLineSensorValue());
            break;
            
        case GCMoveDegrees120:
            return !sonicDelegate(static_cast<DelegateOperation>(isBackFar | isLeftFar)) && leftBackLineSensorValue();
            break;
            
        case GCMoveDegrees_120:
            return !sonicDelegate(static_cast<DelegateOperation>(isBackFar | isRightFar)) && rightBackLineSensorValue();
            break;
            
        case GCMoveDegrees180:
            return !sonicDelegate(isBackFar) && rightBackLineSensorValue() && leftBackLineSensorValue();
            break;
            
        default:
            return false;
    }
    return false;
}
