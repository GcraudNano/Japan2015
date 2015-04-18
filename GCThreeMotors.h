#ifndef __GCThreeMotors__
#define __GCThreeMotors__

#include "GCRobot.h"
#include "GCSharedObjects.h"
#include <limits.h>

class GCThreeMotors {
    
public:
    enum GoBackState {
        RightBack,
        LeftBack,
        StraightBack,
        DoNotBack,
    };
    enum SonicDelegateOperation {
        WhichIsFarther,
        IsRightClose,
        IsLeftClose,
        IsBackClose,
        IsBackFar,
        WhereAmI,
    };
    enum SonicDelegateResult {
        RightIsFarther,
        LeftIsFarther,
        CloseEnoughToPutOut,
        AlmostMiddle,
        TooCloseToOpponentsGoal,
    };
    static constexpr unsigned long factorial(unsigned long x)
    {
        return x > 1 ? x * factorial(x - 1) : 1;
    }
    static constexpr double myPow(double x, unsigned int y)
    {
        return y == 0 ? 1
        :
        y > 1 ? x * myPow(x, y - 1) : x;
    }
    static constexpr double mysin(double x, unsigned int i = 7)
    {
        return i == 0 ? x
        :
        (i % 2 ? -1 : 1) * myPow(x, 2 * i + 1) / factorial(2 * i + 1) + mysin(x, i - 1);
    }
    
    static constexpr int16_t shouldHold = 0;
    static constexpr int16_t shouldFree = INT16_MIN;
    static constexpr int16_t shouldFreeImmediately = INT16_MAX;
    
private:
    typedef enum {
        Degrees0     = 0,
        Degrees112   = 11,
        Degrees_112  = -11,
        Degrees225   = 22,
        Degrees_225  = -22,
        Degrees337   = 34,
        Degrees_337  = -34,
        Degrees450   = 45,
        Degrees_450  = -45,
        Degrees562   = 56,
        Degrees_562  = -56,
        Degrees675   = 67,
        Degrees_675  = -67,
        Degrees787   = 79,
        Degrees_787  = -79,
        Degrees900   = 90,
        Degrees_900  = -90,
        Degrees1012  = 101,
        Degrees_1012 = -101,
        Degrees1125  = 112,
        Degrees_1125 = -112,
        Degrees1237  = 124,
        Degrees_1237 = -124,
        Degrees1350  = 135,
        Degrees_1350 = -135,
        Degrees1462  = 146,
        Degrees_1462 = -146,
        Degrees1500  = 150,
        Degrees_1500 = -150,
        Degrees1575  = 157,
        Degrees_1575 = -157,
        Degrees1687  = 169,
        Degrees_1687 = -169,
        Degrees1800  = 180,
        DegreesNone  = GCThreeMotors::shouldFree,
    } PrivareMoveDegrees;
    static constexpr int fastRightPower   = 150;
    static constexpr int fastRightPower90 = 75;
    static constexpr int maxRightPower    = 90; // 80
    static constexpr int maxRightPower90  = 30; // 40 -> 27
    static constexpr int slowRightPower   = 40; // 40
    static constexpr int slowRightPower90 = 20; // 20 -> 16
    static constexpr int lineRightPower   = 22;
    static constexpr int lineRightPower0  = 30;
    
    static constexpr int fastLeftPower    = 150;
    static constexpr int fastLeftPower90  = 75;
    static constexpr int maxLeftPower     = 90; // 80
    static constexpr int maxLeftPower90   = 30; // 40 -> 27
    static constexpr int slowLeftPower    = 40; // 40
    static constexpr int slowLeftPower90  = 20; // 20 -> 16
    static constexpr int lineLeftPower    = 22;
    static constexpr int lineLeftPower0   = 30;
    
    static constexpr int fastBackPower    = 150;
    static constexpr int fastBackPower90  = 170;
    static constexpr int maxBackPower     = 90; // 80
    static constexpr int maxBackPower90   = 90; // 100 -> 87
    static constexpr int slowBackPower    = 50; // 40
    static constexpr int slowBackPower90  = 50; // 50 -> 46
    static constexpr int lineBackPower    = 26;
    static constexpr int lineBackPower0   = 31;
    
    void (*runRightMotor)(int16_t power);
    void (*runLeftMotor)(int16_t power);
    void (*runBackMotor)(int16_t power);
    GCraudNano::GCLineDegrees (*checkLineSensor)();
    GoBackState (*goBackDelegate)();
    SonicDelegateResult (*sonicDelegate)(SonicDelegateOperation operation);
    GCraudNano::GCMoveDegrees lastLineDegrees;
    GCraudNano::GCBallDegrees lastSpecifiedBallDegrees;
    GCraudNano::GCBallDegrees currentMoveDegrees;
    GCraudNano::GCBallDegrees lastDetectedMoveDegrees;
    bool detectLine();
    bool shouldAddDegrees;
    void (*fixValuesDelegate)();
    int oddCount;
    double integrationValue;
    static constexpr int16_t decideFastRightPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(fastRightPower90 * mysin(safeRadians(degrees + 60)))):
        static_cast<int16_t>(round(fastRightPower   * mysin(safeRadians(degrees + 60))));
    }
    static constexpr int16_t decideRightPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(maxRightPower90 * mysin(safeRadians(degrees + 60)))):
        static_cast<int16_t>(round(maxRightPower   * mysin(safeRadians(degrees + 60))));
    }
    static constexpr int16_t decideSlowRightPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(slowRightPower90 * mysin(safeRadians(degrees + 60)))):
        static_cast<int16_t>(round(slowRightPower   * mysin(safeRadians(degrees + 60))));
    }
    static constexpr int16_t decideLineRightPower(int degrees) {
        return degrees == Degrees0 || degrees == Degrees1800 ?
        static_cast<int16_t>(round(lineRightPower0 * mysin(safeRadians(degrees + 60)))):
        static_cast<int16_t>(round(lineRightPower  * mysin(safeRadians(degrees + 60))));
    }
    
    static constexpr int16_t decideFastLeftPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(fastLeftPower90 * mysin(safeRadians(60 - degrees)))):
        static_cast<int16_t>(round(fastLeftPower   * mysin(safeRadians(60 - degrees))));
    }
    static constexpr int16_t decideLeftPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(maxLeftPower90 * mysin(safeRadians(60 - degrees)))):
        static_cast<int16_t>(round(maxLeftPower   * mysin(safeRadians(60 - degrees))));
    }
    static constexpr int16_t decideSlowLeftPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(slowLeftPower90 * mysin(safeRadians(60 - degrees)))):
        static_cast<int16_t>(round(slowLeftPower   * mysin(safeRadians(60 - degrees))));
    }
    static constexpr int16_t decideLineLeftPower(int degrees) {
        return degrees == Degrees0 || degrees == Degrees1800 ?
        static_cast<int16_t>(round(lineLeftPower0 * mysin(safeRadians(60 - degrees)))):
        static_cast<int16_t>(round(lineLeftPower  * mysin(safeRadians(60 - degrees))));
    }
    
    static constexpr int16_t decideFastBackPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(fastBackPower90 * mysin(safeRadians(degrees - 180)))):
        static_cast<int16_t>(round(fastBackPower   * mysin(safeRadians(degrees - 180))));
    }
    static constexpr int decideBackPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(maxBackPower90 * mysin(safeRadians(degrees - 180)))):
        static_cast<int16_t>(round(maxBackPower   * mysin(safeRadians(degrees - 180))));
    }
    static constexpr int decideSlowBackPower(int degrees) {
        return ((Degrees562 <= abs(degrees) && abs(degrees) <= Degrees1237) || (Degrees_1237 <= abs(degrees) && abs(degrees) <= Degrees_562)) ?
        static_cast<int16_t>(round(slowBackPower90 * mysin(safeRadians(degrees - 180)))):
        static_cast<int16_t>(round(slowBackPower   * mysin(safeRadians(degrees - 180))));
    }
    static constexpr int decideLineBackPower(int degrees) {
        return degrees == Degrees0 || degrees == Degrees1800 ?
        static_cast<int16_t>(round(lineBackPower0 * mysin(safeRadians(degrees - 180)))):
        static_cast<int16_t>(round(lineBackPower  * mysin(safeRadians(degrees - 180))));
    }
    
    static constexpr int PrivateBallDegrees1500  = GCraudNano::GCBallDegrees1800 + 1;
    static constexpr int PrivateBallDegrees_1500 = GCraudNano::GCBallDegrees1800 + 2;
    
public:
    GCThreeMotors();
    void setRunRightMotorDelegate(void (*func)(int16_t power));
    void setRunLeftMotorDelegate(void (*func)(int16_t power));
    void setrunBackMotorDelegate(void (*func)(int16_t power));
    void setLineSensorDelegate(GCraudNano::GCLineDegrees (*func)());
    void setGoBackDelegate(GoBackState (*func)());
    void setSonicDelegate(SonicDelegateResult (*func)(SonicDelegateOperation operation));
    void setFixValuesDelegate(void (*func)());
    void move(
        GCraudNano::GCBallDegrees degrees,
        GCraudNano::GCBallDistance distance,
        GCraudNano::GCBallDegrees tiltValue,
        bool isKeeper
    );
    
};

#endif
