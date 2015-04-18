#ifndef __AppDelegate__
#define __AppDelegate__

#include "GCRobot.h"
#include "GCMotor.h"
#include "GCThreeMotors.h"
#include "GCIRSensor.h"
#include "GCXLineSensors.h"
#include "GCLineSensor.h"
#include "GCSonicSensor.h"
#include "GCThreeSonicSensors.h"
#include "GCCompassSensor.h"
#include "GCSwitch.h"
#include "GCSharedObjects.h"

class AppDelegate {
    static constexpr int  numOfIRSensors = 16;
    static GCMotor        rightMotor;
    static GCMotor        leftMotor;
    static GCMotor        backMotor;
    GCThreeMotors         motors;
    GCIRSensor            irSensors[numOfIRSensors];
    static GCXLineSensors lineSensors;
    static GCLineSensor   rightFrontLineSensor;
    static GCLineSensor   rightBackLineSensor;
    static GCLineSensor   leftFrontLineSensor;
    static GCLineSensor   leftBackLineSensor;
    static GCSonicSensor  rightSonicSensor;
    static GCSonicSensor  leftSonicSensor;
    static GCSonicSensor  backSonicSensor;
    static GCThreeSonicSensors sonicSensors;
    static GCCompassSensor     compassSensor;
    GCSwitch              rightSwitch;
    GCSwitch              leftSwitch;
    GCSwitch              compassSwitch;
    GCraudNano::GCBallDegrees lastTilt;
    void powerDownCPU();
    void idleCPU();
    
public:
    AppDelegate();
    void start();
};

#endif
