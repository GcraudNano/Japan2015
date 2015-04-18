#include "GCSonicSensor.h"
#include <limits.h>

const int GCSonicSensor::timeout  = 10000;
const int GCSonicSensor::interval = 10;

GCSonicSensor::GCSonicSensor(uint8_t _pin, unsigned long _closeToWall)
: pin(_pin), closeToWall(_closeToWall), farFromWall(INT_MAX), rawPin(_pin), value(0)
{
}

GCSonicSensor::GCSonicSensor(uint8_t _pin, unsigned long _closeToWall, unsigned long _farFromWall)
: pin(_pin), closeToWall(_closeToWall), farFromWall(_farFromWall), rawPin(_pin), value(0)
{
}

bool GCSonicSensor::isClose()
{
    return value <= closeToWall;
}

bool GCSonicSensor::isFar()
{
    return value >= farFromWall;
}

unsigned long GCSonicSensor::getValue()
{
    pin.setMode(OUTPUT);
    pin.write(LOW);
    delayMicroseconds(interval);
    pin.write(HIGH);
    delayMicroseconds(interval);
    pin.write(LOW);
    delayMicroseconds(interval);
    pin.setMode(INPUT);
    return value = pin.readPulse(HIGH, timeout) * 17 / 1000;
}
