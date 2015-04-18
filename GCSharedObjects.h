#ifndef __GCSharedObjects__
#define __GCSharedObjects__

#include <limits.h>

namespace GCraudNano {

    typedef enum {
        GCBallDegrees0     = 0,
        GCBallDegrees112   = 1,
        GCBallDegrees_112  = -1,
        GCBallDegrees225   = 2,
        GCBallDegrees_225  = -2,
        GCBallDegrees337   = 3,
        GCBallDegrees_337  = -3,
        GCBallDegrees450   = 4,
        GCBallDegrees_450  = -4,
        GCBallDegrees562   = 5,
        GCBallDegrees_562  = -5,
        GCBallDegrees675   = 6,
        GCBallDegrees_675  = -6,
        GCBallDegrees787   = 7,
        GCBallDegrees_787  = -7,
        GCBallDegrees900   = 8,
        GCBallDegrees_900  = -8,
        GCBallDegrees1012  = 9,
        GCBallDegrees_1012 = -9,
        GCBallDegrees1125  = 10,
        GCBallDegrees_1125 = -10,
        GCBallDegrees1237  = 11,
        GCBallDegrees_1237 = -11,
        GCBallDegrees1350  = 12,
        GCBallDegrees_1350 = -12,
        GCBallDegrees1462  = 13,
        GCBallDegrees_1462 = -13,
        GCBallDegrees1575  = 14,
        GCBallDegrees_1575 = -14,
        GCBallDegrees1687  = 15,
        GCBallDegrees_1687 = -15,
        GCBallDegrees1800  = 16,
        GCBallDegreesNone  = INT_MAX,
    } GCBallDegrees;
    
    typedef enum {
        GCIRSensorDegrees0     = GCBallDegrees0,
        GCIRSensorDegrees225   = GCBallDegrees225,
        GCIRSensorDegrees_225  = GCBallDegrees_225,
        GCIRSensorDegrees450   = GCBallDegrees450,
        GCIRSensorDegrees_450  = GCBallDegrees_450,
        GCIRSensorDegrees675   = GCBallDegrees675,
        GCIRSensorDegrees_675  = GCBallDegrees_675,
        GCIRSensorDegrees900   = GCBallDegrees900,
        GCIRSensorDegrees_900  = GCBallDegrees_900,
        GCIRSensorDegrees1125  = GCBallDegrees1125,
        GCIRSensorDegrees_1125 = GCBallDegrees_1125,
        GCIRSensorDegrees1350  = GCBallDegrees1350,
        GCIRSensorDegrees_1350 = GCBallDegrees_1350,
        GCIRSensorDegrees1575  = GCBallDegrees1575,
        GCIRSensorDegrees_1575 = GCBallDegrees_1575,
        GCIRSensorDegrees1800  = GCBallDegrees1800,
    } GCIRSensorDegrees;
    
    typedef int GCLineDegrees;
    constexpr GCLineDegrees GCLineDegreesNone = 0;
    constexpr GCLineDegrees GCLineDegrees45   = 1 << 0;
    constexpr GCLineDegrees GCLineDegrees_45  = 1 << 1;
    constexpr GCLineDegrees GCLineDegrees135  = 1 << 2;
    constexpr GCLineDegrees GCLineDegrees_135 = 1 << 3;
    
    typedef enum {
        GCMoveDegrees45 = 1,
        GCMoveDegrees_45,
        GCMoveDegrees120,
        GCMoveDegrees_120,
        GCMoveDegrees90,
        GCMoveDegrees_90,
        GCMoveDegrees0,
        GCMoveDegrees180,
        GCMoveDegreesNone,
    } GCMoveDegrees;
    
    typedef enum {
        Far,
        Middle,
        Near,
    } GCBallDistance;
    
}

#endif
