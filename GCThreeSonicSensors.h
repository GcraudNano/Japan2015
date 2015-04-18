#ifndef __GCThreeSonicSensors__
#define __GCThreeSonicSensors__

#include "GCRobot.h"
#include <limits.h>
#include "GCThreeMotors.h"

class GCThreeSonicSensors {
public:
    enum DelegateOperation {
        GetValue,
        IsClose,
        IsFar,
    };
    
private:
    static const unsigned long rapidChange;
    static const unsigned long closeToWall;
    static const unsigned long oppositeIsClose;
    static const unsigned long shouldGoBackDistance;    
    unsigned long (*rightSonic)(DelegateOperation operation);
    unsigned long (*leftSonic)(DelegateOperation operation);
    unsigned long (*backSonic)(DelegateOperation operation);
    bool isRightValid;
    bool isLeftValid;
    bool isBackValid;
    unsigned long rightValue;
    unsigned long leftValue;
    unsigned long backValue;
    
public:
    static constexpr unsigned long invalidValue = ULONG_MAX;
    static const unsigned long stageWidth;
    static const unsigned long stageHeight;
    static const unsigned long halfStageWidth;
    static const unsigned long halfStageHeight;
    GCThreeSonicSensors();
    void setRightSonicDelegate(unsigned long (*func)(DelegateOperation operation));
    void setLeftSonicDelegate(unsigned long (*func)(DelegateOperation operation));
    void setBackSonicDelegate(unsigned long (*func)(DelegateOperation operation));
    void getValues();
    void validateAll();
    bool isRightClose();
    bool isRightFar();
    bool isLeftClose();
    bool isLeftFar();
    bool isBackClose();
    bool isBackFar();
    bool isRightFarther();
    INLINE bool isLeftFarther() {
        return ! isRightFarther();
    }
    GCThreeMotors::GoBackState detectBackState();
    double detectDegreesToGoal();
};

#endif
