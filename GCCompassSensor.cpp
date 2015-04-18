#include "GCCompassSensor.h"
#include "GCWatchdogTimer.h"

#ifdef ARDUINO

#define HMC6352 6352
#define SENSOR_TYPE HMC6352

#if SENSOR_TYPE == HMC6352

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include <Wire.h>
#pragma GCC diagnostic pop
const int i2cAddress = 33; // 0x21

#endif

#endif

const int GCCompassSensor::deadZone = 13;
const double GCCompassSensor::fixNum = 1.4;

GCCompassSensor::GCCompassSensor() : rightFixValueDelegate(NULL), leftFixValueDelegate(NULL), backFixValueDelegate(NULL), offset(0), lastValueForAccel(0), lastGetTime(0), movingAvarage(0), movingValuesPoint(NULL), tiltOffset(0)
{
    movingValues[numOfMovingValues] = endOfMovingValues;
    movingValuesPoint = movingValues;
    this->init();
    
#ifdef ARDUINO
    offset = eeprom_read_byte(reinterpret_cast<unsigned char *>(offsetStartAddress)) << 8 |
             eeprom_read_byte(reinterpret_cast<unsigned char *>(offsetStartAddress + 1));
#endif
    
    int16_t value = readRawValue();
    do {
        *movingValuesPoint = value;
    } while (*++movingValuesPoint != endOfMovingValues);
    movingValuesPoint = movingValues;
    movingAvarage = value;
    lastGetTime = millis();
    lastValueForAccel = static_cast<double>(value) / 10.0;
}

void GCCompassSensor::init()
{
    
#ifdef ARDUINO
    
#   if SENSOR_TYPE == HMC6352
    Wire.begin();
    Wire.beginTransmission(i2cAddress);
    Wire.write('G');
    Wire.write(116); // 0x74
    Wire.write(114); // 0x72
    Wire.endTransmission();
    delayMicroseconds(70);
#   endif
    
#endif
    
}

void GCCompassSensor::setRightFixValueDelegate(void (*func)(int))
{
    rightFixValueDelegate = func;
}

void GCCompassSensor::setLeftFixValueDelegate(void (*func)(int))
{
    leftFixValueDelegate = func;
}

void GCCompassSensor::setBackFixValueDelegate(void (*func)(int))
{
    backFixValueDelegate = func;
}

int16_t GCCompassSensor::readRawValue()
{
    int16_t ret = 0;
    
#ifdef ARDUINO
    
#   if SENSOR_TYPE == HMC6352
    Wire.requestFrom(i2cAddress, 2);
    if (Wire.available() < 2) {
        this->init();
        Wire.requestFrom(i2cAddress, 2);
        if (Wire.available() < 2) {
//            GCRobot::die(F("HMC6352 has dead."));
            GCWatchdogTimer::setDuration(GCWatchdogTimer::Duration_16);
            GCWatchdogTimer::enableReset();
            while (Wire.available() < 2) {
                this->init();
                Wire.requestFrom(i2cAddress, 2);
            }
            GCWatchdogTimer::disableReset();
        }
    }
    ret = -((Wire.read() << 8 | Wire.read()) - 1800);
#   endif
    
#else
    
    static int16_t val = -1710;
    if (val == -1790) {
        val = 1810;
    }
    val -= 10;
    ret = val;
    
#endif
    
    return ret;
}

double GCCompassSensor::readValue()
{
    if (*++movingValuesPoint == endOfMovingValues) {
        movingValuesPoint = movingValues;
    }
    int16_t ret = readRawValue();
    if (movingAvarage > 900 && ret < -900) {
        movingAvarage += -*movingValuesPoint / numOfMovingValues + (ret + 3600) / numOfMovingValues;
        if (movingAvarage > 1800) {
            movingAvarage -= 3600;
        }
    } else if (movingAvarage < -900 && ret > 900) {
        movingAvarage += -*movingValuesPoint / numOfMovingValues + (ret - 3600) / numOfMovingValues;
        if (movingAvarage < -1799) {
            movingAvarage += 3600;
        }
    } else if (movingAvarage > 900 && *movingValuesPoint < -900) {
        movingAvarage += -(*movingValuesPoint + 3600) / numOfMovingValues + ret / numOfMovingValues;
        if (movingAvarage > 1800) {
            movingAvarage -= 3600;
        }
    } else if (movingAvarage < -900 && *movingValuesPoint > 900) {
        movingAvarage += -(*movingValuesPoint - 3600) / numOfMovingValues + ret / numOfMovingValues;
        if (movingAvarage < -1799) {
            movingAvarage += 3600;
        }
    } else {
        movingAvarage += -*movingValuesPoint / numOfMovingValues + ret / numOfMovingValues;
    }
    *movingValuesPoint = ret;
    double value = static_cast<double>(movingAvarage - offset) / 10.0;
    if (value > 180) {
        return value - 360;
    } else if (value < -180) {
        return value + 360;
    }
    
    return value;
}

#define ENABLE_PID

GCraudNano::GCBallDegrees GCCompassSensor::setFixValues(bool isMoving)
{
#ifdef ENABLE_PID
    auto now = millis();
#endif
    
    double compassValue = readValue();
    
#ifdef ENABLE_PID
    double gap = compassValue - lastValueForAccel;
    if (gap > 300) {
        gap -= 360;
    } else if (gap < -300) {
        gap += 360;
    }
    
    double pidValue = (now == lastGetTime ? 0 : -derivationGain * (gap * 1000 /*[ms/s]*/ / (now - lastGetTime))) + (isMoving ? movingPropotionGain : propotionGain) * -compassValue;
    int fixValue = static_cast<int>(round(pidValue));
    lastGetTime = now;
    if (compassValue > 180) {
        lastValueForAccel = compassValue - 360;
    } else if (compassValue < -180) {
        lastValueForAccel = compassValue + 360;
    } else {
        lastValueForAccel = compassValue;
    }
#endif
    
    if (isMoving) {
        rightFixValueDelegate(fixValue);
        backFixValueDelegate(fixValue);
        leftFixValueDelegate(-fixValue);
        return compassValue /= 22.5, static_cast<GCraudNano::GCBallDegrees>(round(compassValue));
    } else if (compassValue > deadZone) {
        /* turn right */
#ifndef ENABLE_PID
        int val = static_cast<int>(fixNum * sqrtf(value) + 0.5);
        rightFixValueDelegate(val);
        backFixValueDelegate(val);
        leftFixValueDelegate(-val);
#else
        rightFixValueDelegate(fixValue);
        backFixValueDelegate(fixValue);
        leftFixValueDelegate(-fixValue);
        return static_cast<GCraudNano::GCBallDegrees>(compassValue / 22.5 + 0.5);
#endif
    } else if (compassValue < -deadZone) {
        /* turn left */
#ifndef ENABLE_PID
        int val = static_cast<int>(fixNum * sqrtf(-value) + 0.5);
#endif
        rightFixValueDelegate(fixValue);
        backFixValueDelegate(fixValue);
        leftFixValueDelegate(-fixValue);
        return static_cast<GCraudNano::GCBallDegrees>(compassValue / 22.5 - 0.5);
    } else {
        rightFixValueDelegate(0);
        leftFixValueDelegate(0);
        backFixValueDelegate(0);
        return GCraudNano::GCBallDegrees0;
    }
}

void GCCompassSensor::calibration()
{
#ifdef ARDUINO
    
#   if SENSOR_TYPE == HMC6352
    static constexpr int calibrationTime = 7000;
    
    Wire.beginTransmission(i2cAddress);
    Wire.write('C');
    Wire.endTransmission();
    delayMicroseconds(10);
    
    delay(calibrationTime);
    
    Wire.beginTransmission(i2cAddress);
    Wire.write('E');
    Wire.endTransmission();
    delayMicroseconds(14000);
#   endif
    
#endif
}

void GCCompassSensor::enterSleep()
{
#ifdef ARDUINO
    
#   if SENSOR_TYPE == HMC6352
    Wire.beginTransmission(i2cAddress);
    Wire.write('S');
    Wire.endTransmission();
    delayMicroseconds(10);
#   endif
    
#endif
}

void GCCompassSensor::exitSleep()
{
#ifdef ARDUINO
    
#   if SENSOR_TYPE == HMC6352
    Wire.beginTransmission(i2cAddress);
    Wire.write('W');
    Wire.endTransmission();
    delayMicroseconds(100);
    this->init();
#   endif
    
#endif
}
