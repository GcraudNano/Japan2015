#ifndef __GCCompassSensor__
#define __GCCompassSensor__

#include "GCRobot.h"
#include "GCSharedObjects.h"
#ifdef ARDUINO
#include <avr/eeprom.h>
#endif

class GCCompassSensor {
    static const int deadZone;
    static const double fixNum;
    void init();
    void (*rightFixValueDelegate)(int fixValue);
    void (*leftFixValueDelegate)(int fixValue);
    void (*backFixValueDelegate)(int fixValue);
    int16_t offset;
    static constexpr auto propotionGain  = 0.5; // 20
    static constexpr auto movingPropotionGain = 0.5; // 1.35 -> 0.8
    static constexpr auto derivationGain = 0.09; // 0.095
    double lastValueForAccel;
    unsigned long lastGetTime;
    static constexpr int numOfMovingValues = 5;
    static constexpr int16_t endOfMovingValues = INT16_MAX;
    int16_t movingAvarage;
    int16_t movingValues[numOfMovingValues + 1];
    int16_t *movingValuesPoint;
    static constexpr int offsetStartAddress = 50;
    
public:
    GCCompassSensor();
    void setRightFixValueDelegate(void (*func)(int fixValue));
    void setLeftFixValueDelegate(void (*func)(int fixValue));
    void setBackFixValueDelegate(void (*func)(int fixValue));
    int16_t readRawValue();
    double readValue();
    INLINE void setOffset() {
        offset = readRawValue();
#ifdef ARDUINO
        int8_t high = offset >> 8;
        int8_t low  = static_cast<int8_t>(offset);
        eeprom_write_byte(reinterpret_cast<unsigned char *>(offsetStartAddress), static_cast<unsigned char>(high));
        eeprom_write_byte(reinterpret_cast<unsigned char *>(offsetStartAddress + 1), static_cast<unsigned char>(low));
#endif
    }
    GCraudNano::GCBallDegrees setFixValues(bool isMoving);
    void calibration();
    void enterSleep();
    void exitSleep();
    double tiltOffset;
};

#endif
