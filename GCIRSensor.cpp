#include "GCIRSensor.h"
using namespace GCraudNano;

const int GCIRSensor::farBase        = 8;
const int GCIRSensor::middleBase     = 14;
const int GCIRSensor::timeOut        = 100;
GCBallDistance GCIRSensor::ballDistance = Far;

int GCIRSensor::median(int *arr, int count)
{
    switch (count) {
        case 1:
            return arr[0];
            
        case 2:
            return (arr[0] + arr[1]) / 2;
            
        default:
#define cast(v) static_cast<const int *>(v)
            qsort(arr, static_cast<size_t>(count), sizeof(int), LAMBDA((const void *a, const void *b), ->, int, {
                return *cast(a) == *cast(b) ? 0 : *cast(a) < *cast(b) ? -1 : 1;
            }));
#undef cast
            if (count % 2 == 0) {
                arr += count / 2 - 1;
                return (*arr + arr[1]) / 2;
            } else {
                return arr[count / 2];
            }
    }
}

GCIRSensor::GCIRSensor(uint8_t _pin, GCIRSensorDegrees _deg1, GCIRSensorDegrees _deg2) : pin(_pin, INPUT), deg1(_deg1), deg2(_deg2), value(0)
{
}

void GCIRSensor::getValues(GCIRSensor *sensors, int count)
{
    GCIRSensor *p = sensors + count;

    do {
        sensors->value = static_cast<int>(sensors->pin.readPulse(LOW, timeOut));
    } while (++sensors != p);
}

GCBallDegrees GCIRSensor::getMedian(GCIRSensor *sensors, int count)
{
    GCIRSensor *p = sensors + count;
    int respondedDegrees[count];
    bool shouldUseDeg1 = true;
    int maxValue = 0;
    int numOfMaxes = 0;
    int numOfRespondedSensors = 0;
    
    do {
        if (sensors->deg1 == GCIRSensorDegrees1800 && sensors->value) {
            shouldUseDeg1 = false;
        }
    } while (++sensors != p);
    sensors -= count;
    
    int value;
    do {
        if ((value = sensors->value)) {
            if (value > maxValue) {
                numOfMaxes = 0;
                respondedDegrees[numOfMaxes++] = shouldUseDeg1 ? sensors->deg1 : sensors->deg2;
                maxValue = value;
            } else if (value == maxValue) {
                respondedDegrees[numOfMaxes++] = shouldUseDeg1 ? sensors->deg1 : sensors->deg2;
            }
            sensors->value = 0;
            numOfRespondedSensors++;
        }
    } while (++sensors != p);
    
    if (numOfMaxes) {
        if (numOfRespondedSensors <= farBase) {
            ballDistance = Far;
        } else if (numOfRespondedSensors <= middleBase) {
            ballDistance = Middle;
        } else {
            ballDistance = Near;
        }
        if (shouldUseDeg1) {
            return static_cast<GCBallDegrees>(median(respondedDegrees, numOfMaxes));
        } else {
            int med = median(respondedDegrees, numOfMaxes);
            if (med <= 0) {
                return static_cast<GCBallDegrees>(med + GCBallDegrees1800);
            } else {
                return static_cast<GCBallDegrees>(med - GCBallDegrees1800);
            }
        }
    }
    return GCBallDegreesNone;
}
