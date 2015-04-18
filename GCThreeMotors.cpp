#include "GCThreeMotors.h"
#include "GCMotor.h"
#include "GCIRSensor.h"
using namespace GCraudNano;

GCThreeMotors::GCThreeMotors() : runRightMotor(NULL), runLeftMotor(NULL), runBackMotor(NULL), checkLineSensor(NULL), goBackDelegate(NULL), sonicDelegate(NULL), lastLineDegrees(GCMoveDegreesNone), lastSpecifiedBallDegrees(GCBallDegreesNone), currentMoveDegrees(GCBallDegreesNone), lastDetectedMoveDegrees(GCBallDegreesNone), shouldAddDegrees(false), fixValuesDelegate(NULL), oddCount(0), integrationValue(1)
{
    
}

void GCThreeMotors::setRunRightMotorDelegate(void (*func)(int16_t))
{
    runRightMotor = func;
}

void GCThreeMotors::setRunLeftMotorDelegate(void (*func)(int16_t))
{
    runLeftMotor = func;
}

void GCThreeMotors::setrunBackMotorDelegate(void (*func)(int16_t))
{
    runBackMotor = func;
}

void GCThreeMotors::setLineSensorDelegate(GCraudNano::GCLineDegrees (*func)())
{
    checkLineSensor = func;
}

void GCThreeMotors::setGoBackDelegate(GCThreeMotors::GoBackState (*func)())
{
    goBackDelegate = func;
}

void GCThreeMotors::setSonicDelegate(GCThreeMotors::SonicDelegateResult (*func)(GCThreeMotors::SonicDelegateOperation))
{
    sonicDelegate = func;
}

void GCThreeMotors::setFixValuesDelegate(void (*func)())
{
    fixValuesDelegate = func;
}

bool GCThreeMotors::detectLine()
{
#define moveAgainstLines(sonic, againstDegrees, lineDegrees)                \
    do {                                                                    \
        if (sonicDelegate(sonic)) {                                         \
            return true;                                                    \
        }                                                                   \
                                                                            \
        int i = GCMotor::numOfSteps;                                        \
        while (i--) {                                                       \
            runRightMotor(shouldFree);                                      \
            runLeftMotor(shouldFree);                                       \
            runBackMotor(shouldFree);                                       \
        }                                                                   \
                                                                            \
        constexpr auto rightPower = decideLineRightPower(againstDegrees);   \
        constexpr auto leftPower  = decideLineLeftPower(againstDegrees);    \
        constexpr auto backPower  = decideLineBackPower(againstDegrees);    \
                                                                            \
        i = GCMotor::numOfSteps;                                            \
        while (i--) {                                                       \
            runRightMotor(rightPower);                                      \
            runLeftMotor(leftPower);                                        \
            runBackMotor(backPower);                                        \
        }                                                                   \
                                                                            \
        lastLineDegrees = lineDegrees;                                      \
    } while (0)
    
    switch (checkLineSensor()) {
        case GCLineDegrees45:
        case GCLineDegrees45 | GCLineDegrees_45 | GCLineDegrees135:
            moveAgainstLines(IsRightClose, Degrees_1350, GCMoveDegrees45);
            break;
            
        case GCLineDegrees_45:
        case GCLineDegrees_45 | GCLineDegrees45 | GCLineDegrees_135:
            moveAgainstLines(IsLeftClose, Degrees1350, GCMoveDegrees_45);
            break;
            
        case GCLineDegrees135:
        case GCLineDegrees135 | GCLineDegrees_135 | GCLineDegrees45:
            moveAgainstLines(IsRightClose, Degrees_450, GCMoveDegrees120);
            break;
            
        case GCLineDegrees_135:
        case GCLineDegrees_135 | GCLineDegrees135 | GCLineDegrees_45:
            moveAgainstLines(IsLeftClose, Degrees450, GCMoveDegrees_120);
            break;
            
        case GCLineDegrees45 | GCLineDegrees_45:
            moveAgainstLines(IsBackClose, Degrees1800, GCMoveDegrees0);
            break;
            
        case GCLineDegrees45 | GCLineDegrees135:
            moveAgainstLines(IsRightClose, Degrees_900, GCMoveDegrees90);
            break;
            
        case GCLineDegrees_45 | GCLineDegrees_135:
            moveAgainstLines(IsLeftClose, Degrees900, GCMoveDegrees_90);
            break;
            
        case GCLineDegrees135 | GCLineDegrees_135:
            moveAgainstLines(IsBackFar, Degrees0, GCMoveDegrees180);
            break;
            
        default:
#ifdef ARDUINO
//            GCRobot::die(F("GCThreeMotors::detectLine() Unreachable Code %d"), deg);
#else
            abort();
#endif
            /* fallthrough */
            
        case GCLineDegrees45 | GCLineDegrees135 | GCLineDegrees_45 | GCLineDegrees_135:
        {
            int i = GCMotor::numOfSteps;
            while (i--) {
                runRightMotor(shouldFree);
                runLeftMotor(shouldFree);
                runBackMotor(shouldFree);
            }
            lastLineDegrees = GCMoveDegreesNone;
        }
            return false;
            
        case GCLineDegreesNone:
            lastLineDegrees = GCMoveDegreesNone;
            return true;
    }
    
#undef moveAgainstLines
#define moveAgainstLines(sonic, againstDegrees, lineDegrees)                \
    do {                                                                    \
        if (sonicDelegate(sonic)) {                                         \
            return true;                                                    \
        }                                                                   \
                                                                            \
        constexpr auto rightPower = decideLineRightPower(againstDegrees);   \
        constexpr auto leftPower  = decideLineLeftPower(againstDegrees);    \
        constexpr auto backPower  = decideLineBackPower(againstDegrees);    \
                                                                            \
        runRightMotor(rightPower);                                          \
        runLeftMotor(leftPower);                                            \
        runBackMotor(backPower);                                            \
                                                                            \
        lastLineDegrees = lineDegrees;                                      \
    } while (0)
    
    const unsigned long moveUntil = millis() + 250;
    
    while (1) {
        fixValuesDelegate();
        switch (checkLineSensor()) {
            case GCLineDegrees45:
            case GCLineDegrees45 | GCLineDegrees_45 | GCLineDegrees135:
                moveAgainstLines(IsRightClose, Degrees_1350, GCMoveDegrees45);
                break;
                
            case GCLineDegrees_45:
            case GCLineDegrees_45 | GCLineDegrees45 | GCLineDegrees_135:
                moveAgainstLines(IsLeftClose, Degrees1350, GCMoveDegrees_45);
                break;
                
            case GCLineDegrees135:
            case GCLineDegrees135 | GCLineDegrees_135 | GCLineDegrees45:
                moveAgainstLines(IsRightClose, Degrees_450, GCMoveDegrees120);
                break;
                
            case GCLineDegrees_135:
            case GCLineDegrees_135 | GCLineDegrees135 | GCLineDegrees_45:
                moveAgainstLines(IsLeftClose, Degrees450, GCMoveDegrees_120);
                break;
                
            case GCLineDegrees45 | GCLineDegrees_45:
                moveAgainstLines(IsBackClose, Degrees1800, GCMoveDegrees0);
                break;
                
            case GCLineDegrees45 | GCLineDegrees135:
                moveAgainstLines(IsRightClose, Degrees_900, GCMoveDegrees90);
                break;
                
            case GCLineDegrees_45 | GCLineDegrees_135:
                moveAgainstLines(IsLeftClose, Degrees900, GCMoveDegrees_90);
                break;
                
            case GCLineDegrees135 | GCLineDegrees_135:
                moveAgainstLines(IsBackFar, Degrees0, GCMoveDegrees180);
                break;
                
            case GCLineDegrees45 | GCLineDegrees135 | GCLineDegrees_45 | GCLineDegrees_135:
            {
                int i = GCMotor::numOfSteps;
                while (i--) {
                    runRightMotor(shouldFree);
                    runLeftMotor(shouldFree);
                    runBackMotor(shouldFree);
                }
                lastLineDegrees = GCMoveDegreesNone;
            }
                return false;
                
            default:
#ifdef ARDUINO
//                GCRobot::die(F("GCThreeMotors::detectLine() Unreachable Code %d"), deg);
#else
                abort();
#endif
                /* fallthrough */
        
            case GCLineDegreesNone:
                if (millis() > moveUntil) return false;
                break;
        }
    }
    
    return false;
#undef moveAgainstLines
}

void GCThreeMotors::move(
    GCBallDegrees degrees,
    GCBallDistance distance,
    GCBallDegrees tiltValue,
    bool isKeeper
)
{
#ifdef __clang__
#pragma mark MoveStep1(Back)
#endif
    /* Step 1. If there is not a ball, just go back */
    if (degrees == GCBallDegreesNone) {
        lastDetectedMoveDegrees = GCBallDegreesNone;
        currentMoveDegrees = GCBallDegreesNone;
        switch (goBackDelegate()) {
            case RightBack:
            if (detectLine()) {
                constexpr auto rightPower = decideRightPower(Degrees_1500);
                constexpr auto leftPower = decideLeftPower(Degrees_1500);
                constexpr auto BackPower = decideBackPower(Degrees_1500);
                runRightMotor(rightPower);
                runLeftMotor(leftPower);
                runBackMotor(BackPower);
            }
                break;
                
            case LeftBack:
            if (detectLine()) {
                constexpr auto rightPower = decideRightPower(Degrees1500);
                constexpr auto leftPower = decideLeftPower(Degrees1500);
                constexpr auto BackPower = decideBackPower(Degrees1500);
                runRightMotor(rightPower);
                runLeftMotor(leftPower);
                runBackMotor(BackPower);
            }
                break;
                
            case StraightBack:
            if (detectLine()) {
                constexpr auto rightPower = decideRightPower(Degrees1800);
                constexpr auto leftPower = decideLeftPower(Degrees1800);
                constexpr auto BackPower = decideBackPower(Degrees1800);
                runRightMotor(rightPower);
                runLeftMotor(leftPower);
                runBackMotor(BackPower);
            }
                break;
                
            case DoNotBack:
                if (sonicDelegate(IsRightClose) && detectLine()) {
                    constexpr auto rightPower = decideRightPower(Degrees900);
                    constexpr auto leftPower = decideLeftPower(Degrees900);
                    constexpr auto BackPower = decideBackPower(Degrees900);
                    runRightMotor(rightPower);
                    runLeftMotor(leftPower);
                    runBackMotor(BackPower);
                    break;
                } else if (sonicDelegate(IsLeftClose) && detectLine()) {
                    constexpr auto rightPower = decideRightPower(Degrees_900);
                    constexpr auto leftPower = decideLeftPower(Degrees_900);
                    constexpr auto BackPower = decideBackPower(Degrees_900);
                    runRightMotor(rightPower);
                    runLeftMotor(leftPower);
                    runBackMotor(BackPower);
                    break;
                }
                /* fallthrough */
#ifdef __clang__
                [[clang::fallthrough]];
#endif
                
            default:
                runRightMotor(shouldFree);
                runLeftMotor(shouldFree);
                runBackMotor(shouldFree);
        }
        return;
    }
    
#ifdef __clang__
#pragma mark MoveStep2(Ignore Odd Input)
#endif
    /* Step 2. Ignore odd input from IR sensors */
    if (lastSpecifiedBallDegrees != GCBallDegreesNone && ((lastSpecifiedBallDegrees > GCBallDegrees0 && degrees == GCBallDegrees_225) || (lastSpecifiedBallDegrees < GCBallDegrees0 && degrees == GCBallDegrees225))) {
        constexpr int maxOddCount = 5;
        if (++oddCount > maxOddCount) lastSpecifiedBallDegrees = degrees;
        return;
    } else {
        oddCount = 0;
        lastSpecifiedBallDegrees = degrees;
    }
    
#ifdef __clang__
#pragma mark MoveStep3(Jump)
#endif
    /* Step 3. Jump to the last step if the ball is far */
    /* i.e. go forward to the ball */
    if (! isKeeper && distance == GCBallDistance::Far) {
        if (abs(degrees) >= GCBallDegrees675) {
            if (! sonicDelegate(IsBackClose)) {
                if (degrees > 0) {
                    degrees = degrees >= GCBallDegrees1237 ? GCBallDegrees1800 : static_cast<GCBallDegrees>(PrivateBallDegrees1500);
                } else {
                    degrees = -degrees >= GCBallDegrees1237 ? GCBallDegrees1800 : static_cast<GCBallDegrees>(PrivateBallDegrees_1500);
                }
                if (tiltValue != GCBallDegrees0) {
                    distance = GCBallDistance::Middle;
                }
                goto movingSwitch;
            }
        } else {
            if (tiltValue != GCBallDegrees0) {
                distance = GCBallDistance::Middle;
            }
            goto movingSwitch;
        }
    }
    
#ifdef __clang__
#pragma mark MoveStep4(Absolute Degrees)
#endif
    /* Step 4. Converting ball degrees to absolute degrees */
    degrees = static_cast<GCBallDegrees>(degrees + tiltValue);
    if (degrees > GCBallDegrees1800) {
        degrees = static_cast<GCBallDegrees>(degrees - GCBallDegrees1800 * 2);
    } else if (degrees < GCBallDegrees_1687) {
        degrees = static_cast<GCBallDegrees>(degrees + GCBallDegrees1800 * 2);
    }
    
#ifdef __clang__
#pragma mark MoveStep5(Force Straight)
#endif
    /* Step 5. If the ball is at almost front(as absolute), there is no risk of own goal */
    /* Skip the step 6 */
    if (! isKeeper) {
        if (degrees == GCBallDegrees0) {
            if (distance == Near && tiltValue == GCBallDegrees0) distance = Middle;
            goto detectActualDegrees;
        } else if (abs(degrees) <= GCBallDegrees225) {
            if (GCBallDegrees0 < degrees && sonicDelegate(WhichIsFarther) == RightIsFarther) {
                goto detectActualDegrees;
            } else if (sonicDelegate(WhichIsFarther) == LeftIsFarther) {
                goto detectActualDegrees;
            }
        }
    }
    
#ifdef __clang__
#pragma mark MoveStep6(Decide Around)
#endif
    /* Step 6. Deciding the degrees to go around to the front of the ball */
    switch (degrees) {
        
        case GCBallDegrees0:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        degrees = GCBallDegrees1800;
                        break;
                        
                    case AlmostMiddle:
                        switch (distance) {
                            case GCBallDistance::Far:
                                degrees = GCBallDegreesNone;
                                break;
                                
                            case GCBallDistance::Middle:
                                degrees = GCBallDegreesNone;
                                break;
                                
                            case GCBallDistance::Near:
                                distance = GCBallDistance::Near;
                                degrees = GCBallDegrees1800;
                                break;
                                
                            default:
                                break;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        switch (distance) {
                            case GCBallDistance::Far:
                                degrees = GCBallDegreesNone;
                                break;
                                
                            case GCBallDistance::Middle:
                                distance = GCBallDistance::Near;
                                degrees = GCBallDegrees0;
                                break;
                                
                            case GCBallDistance::Near:
                                degrees = GCBallDegrees0;
                                break;
                                
                            default:
                                break;
                        }
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = GCBallDegrees0;
            }
            break;
            
        case GCBallDegrees112:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees1687;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees225;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees900 : GCBallDegrees337;
            }
            break;
            
        case GCBallDegrees225:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees1575;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees337;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees1125 : GCBallDegrees450;
            }
            break;
            
        case GCBallDegrees337:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees1462;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees450;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees1237 : GCBallDegrees562;
            }
            break;
            
        case GCBallDegrees450:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees1350;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees562;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees1350 : GCBallDegrees562;
            }
            break;
            
        case GCBallDegrees562:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees1237;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees675;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees1462 : GCBallDegrees562;
            }
            break;
            
        case GCBallDegrees675:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        degrees = distance == GCBallDistance::Near ? GCBallDegrees1350 : GCBallDegrees1125;
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees675; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees675 :
                distance == GCBallDistance::Near ? GCBallDegrees1575 : GCBallDegrees1125;
            }
            break;
            
        case GCBallDegrees787:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        degrees = distance == GCBallDistance::Near ? GCBallDegrees1462 : GCBallDegrees1237;
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees787; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees787 :
                distance == GCBallDistance::Near ? GCBallDegrees1687 : GCBallDegrees1237;
            }
            break;
            
        case GCBallDegrees900:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = sonicDelegate(WhereAmI) == CloseEnoughToPutOut ? GCBallDistance::Middle : GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees1350;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees900; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees900 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Far, GCBallDegrees1800 : GCBallDegrees1350;
            }
            break;
            
        case GCBallDegrees1012:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees1462;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        degrees = GCBallDegrees1800;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees1800 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Far, GCBallDegrees1800 : GCBallDegrees1462;
            }
            
            break;
            
        case GCBallDegrees1125:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = sonicDelegate(WhereAmI) == CloseEnoughToPutOut ? GCBallDistance::Middle : GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees1575;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        degrees = GCBallDegrees1800;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees1800 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Far, GCBallDegrees1800 : GCBallDegrees1575;
            }
            break;
            
        case GCBallDegrees_112:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_1687;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees_225;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees_900 : GCBallDegrees_337;
            }
            break;

        case GCBallDegrees_225:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_1575;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees_337;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees_1125 : GCBallDegrees_450;
            }
            break;
            
        case GCBallDegrees_337:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_1462;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees_450;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees_1237 : GCBallDegrees_562;
            }
            break;
            
        case GCBallDegrees_450:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_1350;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees_562;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees_1350 : GCBallDegrees_562;
            }
            break;
            
        case GCBallDegrees_562:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_1237;
                        break;
                        
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = GCBallDegrees_900;
                        break;
                        
                    case CloseEnoughToPutOut:
                        distance = GCBallDistance::Middle;
                        degrees = GCBallDegrees_675;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = distance == GCBallDistance::Near && ! sonicDelegate(IsBackClose) ? GCBallDegrees_1462 : GCBallDegrees_562;
            }
            break;
            
        case GCBallDegrees_675:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        distance = GCBallDistance::Near;
                        degrees = distance == GCBallDistance::Near ? GCBallDegrees_1350 : GCBallDegrees_1125;
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees_675; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees_675 :
                distance == GCBallDistance::Near ? GCBallDegrees_1575: GCBallDegrees_1125;
            }
            break;
            
        case GCBallDegrees_787:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        degrees = distance == GCBallDistance::Near ? GCBallDegrees_1462 : GCBallDegrees_1237;
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees_787; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees_787 :
                distance == GCBallDistance::Near ? GCBallDegrees_1687 : GCBallDegrees_1237;
            }
            break;
            
        case GCBallDegrees_900:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = sonicDelegate(WhereAmI) == CloseEnoughToPutOut ? GCBallDistance::Middle : GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees_1350;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        /* No Effect */
                        /* degrees = GCBallDegrees_900; */
                        distance = GCBallDistance::Far;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees_900 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Middle, GCBallDegrees1800 : GCBallDegrees_1350;
            }
            break;
            
        case GCBallDegrees_1012:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = sonicDelegate(WhereAmI) == CloseEnoughToPutOut ? GCBallDistance::Middle : GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees_1462;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        degrees = GCBallDegrees1800;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees1800 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Middle, GCBallDegrees1800 : GCBallDegrees_1462;
            }
            break;
            
        case GCBallDegrees_1125:
            if (isKeeper) {
                switch (sonicDelegate(WhereAmI)) {
                    case TooCloseToOpponentsGoal:
                    case AlmostMiddle:
                        if (distance == GCBallDistance::Near) {
                            distance = sonicDelegate(WhereAmI) == CloseEnoughToPutOut ? GCBallDistance::Middle : GCBallDistance::Far;
                            degrees = GCBallDegrees1800;
                        } else {
                            degrees = GCBallDegrees_1575;
                        }
                        break;
                        
                    case CloseEnoughToPutOut:
                        degrees = GCBallDegrees1800;
                        break;
                        
                    default:
                        break;
                }
            } else {
                degrees = sonicDelegate(IsBackClose) ? GCBallDegrees1800 :
                distance == GCBallDistance::Near ? distance = GCBallDistance::Middle, GCBallDegrees1800 : GCBallDegrees_1575;
            }
            break;
            
        case GCBallDegrees_1237:
        case GCBallDegrees_1350:
            if (isKeeper) {
                if (sonicDelegate(WhereAmI) == CloseEnoughToPutOut) {
                    distance = GCBallDistance::Far;
                    degrees = GCBallDegrees1800;
                    break;
                }
            } else if (sonicDelegate(IsBackClose)) {
                degrees = GCBallDegrees1800;
                break;
            }
            /* fallthrough */
#ifdef __clang__
            [[clang::fallthrough]];
#endif
            
        case GCBallDegrees1237:
        case GCBallDegrees1350:
            if (isKeeper) {
                if (distance == GCBallDistance::Near) distance = GCBallDistance::Middle;
                degrees = GCBallDegrees1800;
            } else {
                if (distance == GCBallDistance::Near) distance = GCBallDistance::Middle;
                degrees = GCBallDegrees1800;
            }
            break;
            
        case GCBallDegrees1462:
        case GCBallDegrees1575:
        case GCBallDegrees1687:
            if (isKeeper) {
                if (distance == GCBallDistance::Near) {
                    distance = GCBallDistance::Middle;
                    if (sonicDelegate(IsLeftClose)) {
                        degrees = GCBallDegrees_900;
                    } else if (sonicDelegate(IsRightClose)) {
                        degrees = GCBallDegrees900;
                    } else {
                        degrees = static_cast<GCBallDegrees>(PrivateBallDegrees_1500);
                    }
                } else {
                    degrees = sonicDelegate(IsRightClose) ? GCBallDegrees1800 : GCBallDegrees_1350;
                }
            } else {
                if (distance == GCBallDistance::Near) {
                    distance = GCBallDistance::Middle;
                    if (sonicDelegate(IsLeftClose)) {
                        degrees = GCBallDegrees_900;
                    } else if (sonicDelegate(IsRightClose)) {
                        degrees = GCBallDegrees900;
                    } else {
                        degrees = static_cast<GCBallDegrees>(PrivateBallDegrees_1500);
                    }
                } else {
                    degrees = sonicDelegate(IsRightClose) ? GCBallDegrees1800 : GCBallDegrees_1350;
                }
            }
            break;
            
        case GCBallDegrees_1462:
        case GCBallDegrees_1575:
        case GCBallDegrees_1687:
            if (isKeeper) {
                if (distance == GCBallDistance::Near) {
                    distance = GCBallDistance::Middle;
                    if (sonicDelegate(IsRightClose)) {
                        degrees = GCBallDegrees900;
                    } else if (sonicDelegate(IsLeftClose)) {
                        degrees = GCBallDegrees_900;
                    } else {
                        degrees = static_cast<GCBallDegrees>(PrivateBallDegrees1500);
                    }
                } else {
                    degrees = sonicDelegate(IsLeftClose) ? GCBallDegrees1800 : GCBallDegrees1350;
                }
            } else {
                if (distance == GCBallDistance::Near) {
                    distance = GCBallDistance::Middle;
                    if (sonicDelegate(IsRightClose)) {
                        degrees = GCBallDegrees900;
                    } else if (sonicDelegate(IsLeftClose)) {
                        degrees = GCBallDegrees_900;
                    } else {
                        degrees = static_cast<GCBallDegrees>(PrivateBallDegrees1500);
                    }
                } else {
                    degrees = sonicDelegate(IsLeftClose) ? GCBallDegrees1800 : GCBallDegrees1350;
                }
            }
            break;
            
        case GCBallDegrees1800:
            if (isKeeper) {
                if (distance == GCBallDistance::Near) distance = GCBallDistance::Middle;
                if (sonicDelegate(WhichIsFarther) == RightIsFarther) {
                    degrees = static_cast<GCBallDegrees>(PrivateBallDegrees_1500);
                } else {
                    degrees = static_cast<GCBallDegrees>(PrivateBallDegrees1500);
                }
            } else {
                if (distance == GCBallDistance::Near) distance = GCBallDistance::Middle;
                if (sonicDelegate(WhichIsFarther) == RightIsFarther) {
                    degrees = static_cast<GCBallDegrees>(PrivateBallDegrees_1500);
                } else {
                    degrees = static_cast<GCBallDegrees>(PrivateBallDegrees1500);
                }
            }
            break;
            
        default:
#ifdef ARDUINO
            GCRobot::die(F("GCThreeMotors::move() Unreachable code 1"));
#else
            abort();
#endif
    }
    
#ifdef __clang__
#pragma mark MoveStep7(Actual Degrees)
#endif
    /* Step 7. Detect the actual moving degrees */
detectActualDegrees:
    if (degrees != GCBallDegreesNone && distance != GCBallDistance::Far) {
        degrees = static_cast<GCBallDegrees>(degrees - tiltValue);
        if (degrees > GCBallDegrees1800) {
            degrees = static_cast<GCBallDegrees>(degrees - GCBallDegrees1800 * 2);
        } else if (degrees < GCBallDegrees_1687) {
            degrees = static_cast<GCBallDegrees>(degrees + GCBallDegrees1800 * 2);
        }
    }
    
#ifdef __clang__
#pragma mark MoveStep8(Move Around)
#endif
    /* Step 8. Move around to the front of the ball */
#define moveWithDegrees(lineCheckDegrees, noLineDegrees, sonic)                                                     \
    do {                                                                                                            \
        if (lastLineDegrees != GCMoveDegreesNone) {                                                                 \
            if (lastLineDegrees == lineCheckDegrees) {                                                              \
                if (checkLineSensor() == 0) {                                                                       \
                    runRightMotor(shouldFree);                                                                      \
                    runLeftMotor(shouldFree);                                                                       \
                    runBackMotor(shouldFree);                                                                       \
                } else {                                                                                            \
                    detectLine();                                                                                   \
                }                                                                                                   \
            } else {                                                                                                \
                lastLineDegrees = GCMoveDegreesNone;                                                                \
            }                                                                                                       \
        } else if (detectLine()) {                                                                                  \
            if (sonic) {                                                                                            \
                constexpr auto rightPower = decideSlowRightPower(noLineDegrees);                                    \
                constexpr auto leftPower  = decideSlowLeftPower(noLineDegrees);                                     \
                constexpr auto backPower  = decideSlowBackPower(noLineDegrees);                                     \
                                                                                                                    \
                runRightMotor(rightPower);                                                                          \
                runLeftMotor(leftPower);                                                                            \
                runBackMotor(backPower);                                                                            \
            } else {                                                                                                \
                switch (distance) {                                                                                 \
                    case Far:                                                                                       \
                    {                                                                                               \
                        constexpr auto rightPower = decideFastRightPower(noLineDegrees);                            \
                        constexpr auto leftPower  = decideFastLeftPower(noLineDegrees);                             \
                        constexpr auto backPower  = decideFastBackPower(noLineDegrees);                             \
                                                                                                                    \
                        runRightMotor(static_cast<int16_t>(static_cast<double>(rightPower) * integrationValue));    \
                        runLeftMotor(static_cast<int16_t>(static_cast<double>(leftPower) * integrationValue));      \
                        runBackMotor(static_cast<int16_t>(static_cast<double>(backPower) * integrationValue));      \
                    }                                                                                               \
                        break;                                                                                      \
                                                                                                                    \
                    case Middle:                                                                                    \
                    {                                                                                               \
                        constexpr auto rightPower = decideRightPower(noLineDegrees);                                \
                        constexpr auto leftPower  = decideLeftPower(noLineDegrees);                                 \
                        constexpr auto backPower  = decideBackPower(noLineDegrees);                                 \
                                                                                                                    \
                        runRightMotor(static_cast<int16_t>(static_cast<double>(rightPower) * integrationValue));    \
                        runLeftMotor(static_cast<int16_t>(static_cast<double>(leftPower) * integrationValue));      \
                        runBackMotor(static_cast<int16_t>(static_cast<double>(backPower) * integrationValue));      \
                    }                                                                                               \
                        break;                                                                                      \
                                                                                                                    \
                    case Near:                                                                                      \
                    {                                                                                               \
                        constexpr auto rightPower = decideSlowRightPower(noLineDegrees);                            \
                        constexpr auto leftPower  = decideSlowLeftPower(noLineDegrees);                             \
                        constexpr auto backPower  = decideSlowBackPower(noLineDegrees);                             \
                                                                                                                    \
                        runRightMotor(static_cast<int16_t>(static_cast<double>(rightPower) * integrationValue));    \
                        runLeftMotor(static_cast<int16_t>(static_cast<double>(leftPower) * integrationValue));      \
                        runBackMotor(static_cast<int16_t>(static_cast<double>(backPower) * integrationValue));      \
                    }                                                                                               \
                        break;                                                                                      \
                }                                                                                                   \
            }                                                                                                       \
        } else {                                                                                                    \
            integrationValue = 1;                                                                                   \
        }                                                                                                           \
    } while (0)

movingSwitch:
    
    /* But first of all, do not attempt to change directions rapidly */
    constexpr GCBallDegrees rapidBase = GCBallDegrees450;
    if (currentMoveDegrees != GCBallDegreesNone && degrees != GCBallDegreesNone) {
        if (currentMoveDegrees >= GCBallDegrees0) {
            currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees - GCBallDegrees1800);
        } else {
            currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees + GCBallDegrees1800);
        }
        if (abs(degrees - currentMoveDegrees) >= rapidBase - GCBallDegrees0) {
            if (degrees >= GCBallDegrees0 && currentMoveDegrees >= GCBallDegrees0) {
                if (degrees > currentMoveDegrees) {
                    currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees + rapidBase - GCBallDegrees0);
                    if (currentMoveDegrees > GCBallDegrees1800) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees_1687 - 1 + currentMoveDegrees - GCBallDegrees1800);
                    }
                } else {
                    currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees - rapidBase + GCBallDegrees0);
                    if (currentMoveDegrees < GCBallDegrees_1687) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees1800 + 1 + currentMoveDegrees - GCBallDegrees_1687);
                    }
                }
            } else if (degrees < GCBallDegrees0 && currentMoveDegrees < GCBallDegrees0) {
                if (degrees < currentMoveDegrees) {
                    currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees - rapidBase - GCBallDegrees0);
                    if (currentMoveDegrees > GCBallDegrees1800) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees_1687 - 1 + currentMoveDegrees - GCBallDegrees1800);
                    }
                } else {
                    currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees + rapidBase + GCBallDegrees0);
                    if (currentMoveDegrees < GCBallDegrees_1687) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees1800 + 1 + currentMoveDegrees - GCBallDegrees_1687);
                    }
                }
            } else {
                int degreesDistance = abs(degrees - currentMoveDegrees);
                if (degreesDistance > abs(GCBallDegrees900 - GCBallDegrees_900)) {
                    if (currentMoveDegrees > GCBallDegrees0) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees + rapidBase - GCBallDegrees0);
                        if (currentMoveDegrees > GCBallDegrees1800) {
                            currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees_1687 - 1 + currentMoveDegrees - GCBallDegrees1800);
                        }
                    } else {
                        currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees - rapidBase + GCBallDegrees0);
                        if (currentMoveDegrees < GCBallDegrees_1687) {
                            currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees1800 + 1 + currentMoveDegrees - GCBallDegrees_1687);
                        }
                    }
                } else {
                    if (currentMoveDegrees < degrees) {
                        currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees + rapidBase - GCBallDegrees0);
                        if (currentMoveDegrees > GCBallDegrees1800) {
                            currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees_1687 - 1 + currentMoveDegrees - GCBallDegrees1800);
                        }
                    } else {
                        currentMoveDegrees = static_cast<GCBallDegrees>(currentMoveDegrees - rapidBase + GCBallDegrees0);
                        if (currentMoveDegrees < GCBallDegrees_1687) {
                            currentMoveDegrees = static_cast<GCBallDegrees>(GCBallDegrees1800 + 1 + currentMoveDegrees - GCBallDegrees_1687);
                        }
                    }
                }
            }
        } else {
            constexpr double maxIntegrationValue = 2.0;
            if (currentMoveDegrees == degrees && degrees == GCBallDegrees1800) {
                if (integrationValue < maxIntegrationValue) {
                    constexpr double integrationStep = 0.04;
                    integrationValue += integrationStep;
                }
            } else {
                integrationValue = 1;
            }
            currentMoveDegrees = degrees;
        }
    }

    switch (static_cast<int>(degrees)) {
            
        case GCBallDegrees0:
            moveWithDegrees(GCMoveDegrees0, Degrees0, sonicDelegate(IsBackFar));
            break;
            
        case GCBallDegrees112:
            moveWithDegrees(GCMoveDegrees0, Degrees112, sonicDelegate(IsBackFar));
            break;
            
        case GCBallDegrees_112:
            moveWithDegrees(GCMoveDegrees0, Degrees_112, sonicDelegate(IsBackFar));
            break;
            
        case GCBallDegrees225:
            moveWithDegrees(GCMoveDegrees0, Degrees225, sonicDelegate(IsBackFar));
            break;
            
        case GCBallDegrees_225:
            moveWithDegrees(GCMoveDegrees0, Degrees_225, sonicDelegate(IsBackFar));
            break;
            
        case GCBallDegrees337:
            moveWithDegrees(GCMoveDegrees45, Degrees337, sonicDelegate(IsBackFar) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_337:
            moveWithDegrees(GCMoveDegrees_45, Degrees_337, sonicDelegate(IsBackFar) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees450:
            moveWithDegrees(GCMoveDegrees45, Degrees450, sonicDelegate(IsBackFar) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_450:
            moveWithDegrees(GCMoveDegrees_45, Degrees_450, sonicDelegate(IsBackFar) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees562:
            moveWithDegrees(GCMoveDegrees45, Degrees562, sonicDelegate(IsBackFar) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_562:
            moveWithDegrees(GCMoveDegrees_45, Degrees_562, sonicDelegate(IsBackFar) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees675:
            moveWithDegrees(GCMoveDegrees90, Degrees675, sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_675:
            moveWithDegrees(GCMoveDegrees_90, Degrees_675, sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees787:
            moveWithDegrees(GCMoveDegrees90, Degrees787, sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_787:
            moveWithDegrees(GCMoveDegrees_90, Degrees_787, sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees900:
            moveWithDegrees(GCMoveDegrees90, Degrees900, sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_900:
            moveWithDegrees(GCMoveDegrees_90, Degrees_900, sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1012:
            moveWithDegrees(GCMoveDegrees90, Degrees1012, sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_1012:
            moveWithDegrees(GCMoveDegrees_90, Degrees_1012, sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1125:
            moveWithDegrees(GCMoveDegrees90, Degrees1125, sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_1125:
            moveWithDegrees(GCMoveDegrees_90, Degrees_1125, sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1237:
            moveWithDegrees(GCMoveDegrees120, Degrees1237, sonicDelegate(IsBackClose) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_1237:
            moveWithDegrees(GCMoveDegrees_120, Degrees_1237, sonicDelegate(IsBackClose) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1350:
            moveWithDegrees(GCMoveDegrees120, Degrees1350, sonicDelegate(IsBackClose) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_1350:
            moveWithDegrees(GCMoveDegrees_120, Degrees_1350, sonicDelegate(IsBackClose) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1462:
            moveWithDegrees(GCMoveDegrees120, Degrees1462, sonicDelegate(IsBackClose) || sonicDelegate(IsLeftClose));
            break;
            
        case GCBallDegrees_1462:
            moveWithDegrees(GCMoveDegrees_120, Degrees_1462, sonicDelegate(IsBackClose) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegrees1575:
            moveWithDegrees(GCMoveDegrees180, Degrees1575, sonicDelegate(IsBackClose));
            break;
            
        case GCBallDegrees_1575:
            moveWithDegrees(GCMoveDegrees180, Degrees_1575, sonicDelegate(IsBackClose));
            break;
            
        case GCBallDegrees1687:
            moveWithDegrees(GCMoveDegrees180, Degrees1687, sonicDelegate(IsBackClose));
            break;
            
        case GCBallDegrees_1687:
            moveWithDegrees(GCMoveDegrees180, Degrees_1687, sonicDelegate(IsBackClose));
            break;
            
        case GCBallDegrees1800:
            moveWithDegrees(GCMoveDegrees180, Degrees1800, sonicDelegate(IsBackClose));
            break;
            
        case PrivateBallDegrees1500:
            moveWithDegrees(GCMoveDegrees120, Degrees1500, sonicDelegate(IsBackClose) || sonicDelegate(IsLeftClose));
            break;
            
        case PrivateBallDegrees_1500:
            moveWithDegrees(GCMoveDegrees_120, Degrees_1500, sonicDelegate(IsBackClose) || sonicDelegate(IsRightClose));
            break;
            
        case GCBallDegreesNone:
            runRightMotor(shouldFree);
            runLeftMotor(shouldFree);
            runBackMotor(shouldFree);
            break;
            
        default:
#ifdef ARDUINO
            GCRobot::die(F("GCThreeMotors::move() Unreachable code 2 with value %d"), currentMoveDegrees);
#else
            abort();
#endif
            
    }
    
#undef moveWithDegrees
}
