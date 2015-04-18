#include "GCThreeSonicSensors.h"

const unsigned long GCThreeSonicSensors::rapidChange = 20;
const unsigned long GCThreeSonicSensors::closeToWall = 40;
const unsigned long GCThreeSonicSensors::oppositeIsClose = stageWidth - closeToWall;
const unsigned long GCThreeSonicSensors::stageWidth  = 115; // actually 123
const unsigned long GCThreeSonicSensors::stageHeight = 183;
const unsigned long GCThreeSonicSensors::shouldGoBackDistance = 32;
const unsigned long GCThreeSonicSensors::halfStageWidth = GCThreeSonicSensors::stageWidth / 2;
const unsigned long GCThreeSonicSensors::halfStageHeight = GCThreeSonicSensors::stageHeight / 2;

GCThreeSonicSensors::GCThreeSonicSensors() : rightSonic(NULL), leftSonic(NULL), backSonic(NULL), isRightValid(false), isLeftValid(false), isBackValid(false), rightValue(0), leftValue(0), backValue(0)
{
    
}

void GCThreeSonicSensors::getValues()
{
    unsigned long newRightValue = rightSonic(GetValue);
    unsigned long newLeftValue  = leftSonic(GetValue);
    unsigned long newBackValue  = backSonic(GetValue);
    
    if ((newRightValue + newLeftValue) >= stageWidth) {
        isRightValid = true;
        isLeftValid  = true;
    } else {
        unsigned long rightChange = newRightValue - rightValue;
        if (abs(rightChange) >= rapidChange) {
            isRightValid = rightChange > 0 ? true : false;
        }
        unsigned long leftChange = newLeftValue - leftValue;
        if (abs(leftChange) >= rapidChange) {
            isLeftValid = leftChange > 0 ? true : false;
        }
    }
    unsigned long backChange = newBackValue - backValue;
    if (abs(backChange) >= rapidChange) {
        isBackValid = backChange > 0 ? true : false;
    }
    
    rightValue = newRightValue;
    leftValue  = newLeftValue;
    backValue  = newBackValue;
}

void GCThreeSonicSensors::setRightSonicDelegate(unsigned long (*func)(DelegateOperation operation))
{
    rightSonic = func;
}

void GCThreeSonicSensors::setLeftSonicDelegate(unsigned long (*func)(DelegateOperation operation))
{
    leftSonic = func;
}

void GCThreeSonicSensors::setBackSonicDelegate(unsigned long (*func)(DelegateOperation operation))
{
    backSonic = func;
}

void GCThreeSonicSensors::validateAll()
{
    isRightValid = true;
    isLeftValid  = true;
    isBackValid  = true;
}

bool GCThreeSonicSensors::isRightClose()
{
    if (isRightValid) {
        return rightSonic(IsClose);
    } else if (isLeftValid) {
        return leftSonic(IsFar);
    }
    return false;
}

bool GCThreeSonicSensors::isRightFar()
{
    if (isRightValid) {
        return rightSonic(IsFar);
    } else if (isLeftValid) {
        return leftSonic(IsClose);
    }
    return false;
}

bool GCThreeSonicSensors::isLeftClose()
{
    if (isLeftValid) {
        return leftSonic(IsClose);
    } else if (isRightValid) {
        return rightSonic(IsFar);
    }
    return false;
}

bool GCThreeSonicSensors::isLeftFar()
{
    if (isLeftValid) {
        return leftSonic(IsFar);
    } else if (isRightValid) {
        return rightSonic(IsClose);
    }
    return false;
}

bool GCThreeSonicSensors::isBackClose()
{
    return isBackValid && backSonic(IsClose);
}

bool GCThreeSonicSensors::isBackFar()
{
    return isBackValid && backSonic(IsFar);
}

bool GCThreeSonicSensors::isRightFarther()
{
    if (isRightValid) {
        if (isLeftValid) {
            return rightValue >= leftValue;
        } else {
            return rightValue >= halfStageWidth;
        }
    }
    return isLeftValid && leftValue <= halfStageWidth;
}

GCThreeMotors::GoBackState GCThreeSonicSensors::detectBackState()
{
    if (backValue >= shouldGoBackDistance && (isRightValid || isLeftValid)) {
        if (isLeftClose()) {
            return GCThreeMotors::RightBack;
        } else if (isRightClose()) {
            return GCThreeMotors::LeftBack;
        } else {
            return GCThreeMotors::StraightBack;
        }
    }
    return GCThreeMotors::DoNotBack;
}

double GCThreeSonicSensors::detectDegreesToGoal()
{
    constexpr long goalToWall = 30;
    if (isBackValid) {
        if (isRightValid) {
            if (isLeftValid) {
                long gyap = static_cast<long>(rightValue) - static_cast<long>(leftValue);
                if (gyap > 0) {
                    return -degrees(atan2(gyap, stageHeight - goalToWall - backValue));
                } else {
                    return degrees(atan2(-gyap, stageHeight - goalToWall - backValue));
                }
            } else {
                long gyap = static_cast<long>(halfStageWidth) - static_cast<long>(rightValue);
                if (gyap > 0) {
                    return degrees(atan2(gyap, stageHeight - goalToWall - backValue));
                } else {
                    return -degrees(atan2(-gyap, stageHeight - goalToWall - backValue));
                }
            }
        } else if (isLeftValid) {
            long gyap = static_cast<long>(halfStageWidth) - static_cast<long>(leftValue);
            if (gyap > 0) {
                return -degrees(atan2(gyap, stageHeight - goalToWall - backValue));
            } else {
                return degrees(atan2(-gyap, stageHeight - goalToWall - backValue));
            }
        }
    }
    return 0;
}
