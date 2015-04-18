#ifndef __GCWatchdogTimer__
#define __GCWatchdogTimer__

#include "GCRobot.h"

#ifndef ARDUINO
#define WDIF    7
#define WDIE    6
#define WDP3    5
#define WDCE    4
#define WDE     3
#define WDP2    2
#define WDP1    1
#define WDP0    0
#endif

class GCWatchdogTimer {
public:
    typedef enum {
        Duration_16   = 0,
        Duration_32   = 1 << WDP0,
        Duration_64   = 1 << WDP1,
        Duration_125  = 1 << WDP1 | 1 << WDP0,
        Duration_250  = 1 << WDP2,
        Duration_500  = 1 << WDP2 | 1 << WDP0,
        Duration_1000 = 1 << WDP2 | 1 << WDP1,
        Duration_2000 = 1 << WDP2 | 1 << WDP1 | 1 << WDP0,
        Duration_4000 = 1 << WDP3,
        Duration_8000 = 1 << WDP3 | 1 << WDP0,
    } GCWatchdogTimerDuration;
    
    static void init();
    static void setDuration(GCWatchdogTimerDuration duration);
    static void enableInterrupt();
    static void disableInterrupt();
    static void enableReset();
    static void disableReset();
};

#endif
