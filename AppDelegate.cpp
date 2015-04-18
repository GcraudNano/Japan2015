#include "AppDelegate.h"
#include "GCSharedObjects.h"
#include "GCWatchdogTimer.h"
#ifdef ARDUINO
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#else
const uint8_t A0 = 0;
const uint8_t A1 = 1;
const uint8_t A2 = 2;
const uint8_t A3 = 3;
#endif
using namespace GCraudNano;

GCMotor        AppDelegate::rightMotor(4, 6, 5);
GCMotor        AppDelegate::leftMotor(3, 14, 2);
GCMotor        AppDelegate::backMotor(9, 7, 8);
GCXLineSensors AppDelegate::lineSensors;
GCLineSensor   AppDelegate::rightFrontLineSensor(A3, 100, GCLineDegrees_45);
GCLineSensor   AppDelegate::rightBackLineSensor(A2, 130, GCLineDegrees_135);
GCLineSensor   AppDelegate::leftFrontLineSensor(A1, 100, GCLineDegrees45);
GCLineSensor   AppDelegate::leftBackLineSensor(A0, 100, GCLineDegrees135);
GCSonicSensor  AppDelegate::rightSonicSensor(51, 40, 87);
GCSonicSensor  AppDelegate::leftSonicSensor(48, 40, 87);
GCSonicSensor  AppDelegate::backSonicSensor(52, 40, 130);
GCThreeSonicSensors AppDelegate::sonicSensors;
GCCompassSensor AppDelegate::compassSensor;

//#define PRINT_IR
//#define LOGGING
//#define DEBUGGING

AppDelegate::AppDelegate() :
    motors(),
    irSensors({
        GCIRSensor(19, GCIRSensorDegrees0,     GCIRSensorDegrees1800),
        GCIRSensor(26, GCIRSensorDegrees225,   GCIRSensorDegrees_1575),
        GCIRSensor(15, GCIRSensorDegrees450,   GCIRSensorDegrees_1350),
        GCIRSensor(18, GCIRSensorDegrees675,   GCIRSensorDegrees_1125),
        GCIRSensor(16, GCIRSensorDegrees900,   GCIRSensorDegrees_900),
        GCIRSensor(17, GCIRSensorDegrees1125,  GCIRSensorDegrees_675),
        GCIRSensor(22, GCIRSensorDegrees1350,  GCIRSensorDegrees_450),
        GCIRSensor(24, GCIRSensorDegrees1575,  GCIRSensorDegrees_225),
        GCIRSensor(12, GCIRSensorDegrees1800,  GCIRSensorDegrees0),
        GCIRSensor(10, GCIRSensorDegrees_1575, GCIRSensorDegrees225),
        GCIRSensor(11, GCIRSensorDegrees_1350, GCIRSensorDegrees450),
        GCIRSensor(59, GCIRSensorDegrees_1125, GCIRSensorDegrees675),
        GCIRSensor(60, GCIRSensorDegrees_900,  GCIRSensorDegrees900),
        GCIRSensor(58, GCIRSensorDegrees_675,  GCIRSensorDegrees1125),
        GCIRSensor(61, GCIRSensorDegrees_450,  GCIRSensorDegrees1350),
        GCIRSensor(13, GCIRSensorDegrees_225,  GCIRSensorDegrees1575),
    }),
    rightSwitch(53, LOW),
    leftSwitch(49, LOW),
    compassSwitch(39, LOW),
    lastTilt(GCBallDegrees0)
{
#if (defined(PRINT_IR) || defined(LOGGING) || defined(DEBUGGING)) && defined(ARDUINO)
    Serial.begin(9600);
#endif
    
#define setMotorPower(m, p)                         \
    do {                                            \
        switch (p) {                                \
        case GCThreeMotors::shouldHold:             \
            m.hold();                               \
            break;                                  \
        case GCThreeMotors::shouldFree:             \
            m.free();                               \
            break;                                  \
        case GCThreeMotors::shouldFreeImmediately:  \
            m.freeImmediately();                    \
            break;                                  \
        default:                                    \
            m.forward(p);                           \
            break;                                  \
        }                                           \
    } while (0)

    motors.setRunRightMotorDelegate(LAMBDA((int16_t degrees), ->, void, {
        setMotorPower(
              rightMotor,
              degrees
        );
    }));
    motors.setRunLeftMotorDelegate(LAMBDA((int16_t degrees), ->, void, {
        setMotorPower(
              leftMotor,
              degrees
        );
    }));
    motors.setrunBackMotorDelegate(LAMBDA((int16_t degrees), ->, void, {
        setMotorPower(
              backMotor,
              degrees
        );
    }));
#undef setMotorPower
    
    motors.setGoBackDelegate(LAMBDA((), ->, GCThreeMotors::GoBackState, {
        if (rightMotor.fixPower == 0) {
            return sonicSensors.detectBackState();
        } else {
            return GCThreeMotors::DoNotBack;
        }
    }));
    
    lineSensors.setRightFrontLineSensorDelegate(LAMBDA((), ->, GCraudNano::GCLineDegrees, {
        return rightFrontLineSensor.isWhite();
    }));
    lineSensors.setRightBackLineSensorDelegate(LAMBDA((), ->, GCraudNano::GCLineDegrees, {
        return rightBackLineSensor.isWhite();
    }));
    lineSensors.setLeftFrontLineSensorDelegate(LAMBDA((), ->, GCraudNano::GCLineDegrees, {
        return leftFrontLineSensor.isWhite();
    }));
    lineSensors.setLeftBackLineSensorDelegate(LAMBDA((), ->, GCraudNano::GCLineDegrees, {
        return leftBackLineSensor.isWhite();
    }));
    lineSensors.setSonicDelegate(LAMBDA((GCXLineSensors::DelegateOperation operation), ->, bool, {
        if (operation & GCXLineSensors::isRightFar  && sonicSensors.isRightFar() ) return true;
        if (operation & GCXLineSensors::isLeftFar   && sonicSensors.isLeftFar()  ) return true;
        if (operation & GCXLineSensors::isBackFar   && sonicSensors.isBackFar()  ) return true;
        if (operation & GCXLineSensors::isBackClose && sonicSensors.isBackClose()) return true;
        return false;
    }));
    
    motors.setLineSensorDelegate(LAMBDA((), ->, GCraudNano::GCLineDegrees, {
        return lineSensors.lineDegrees();
    }));
    motors.setSonicDelegate(
        LAMBDA((GCThreeMotors::SonicDelegateOperation operation), ->, GCThreeMotors::SonicDelegateResult, {
        switch (operation) {
            case GCThreeMotors::WhichIsFarther:
                return sonicSensors.isRightFarther() ? GCThreeMotors::RightIsFarther : GCThreeMotors::LeftIsFarther;
                
            case GCThreeMotors::IsRightClose:
                return static_cast<GCThreeMotors::SonicDelegateResult>(sonicSensors.isRightClose());
                
            case GCThreeMotors::IsLeftClose:
                return static_cast<GCThreeMotors::SonicDelegateResult>(sonicSensors.isLeftClose());
                
            case GCThreeMotors::IsBackClose:
                return static_cast<GCThreeMotors::SonicDelegateResult>(sonicSensors.isBackClose());
                
            case GCThreeMotors::IsBackFar:
                return static_cast<GCThreeMotors::SonicDelegateResult>(sonicSensors.isBackFar());
                
            case GCThreeMotors::WhereAmI:
            {
                auto value = backSonicSensor.value;
                if (value <= 35) {
                    return GCThreeMotors::CloseEnoughToPutOut;
                } else if (value <= 80) {
                    return GCThreeMotors::AlmostMiddle;
                } else {
                    return GCThreeMotors::TooCloseToOpponentsGoal;
                }
            }
                break;
                
            default:
                GCRobot::die(F("SonicDelegate Unreachable Code"));
                return static_cast<GCThreeMotors::SonicDelegateResult>(0);
        }
    }));
    motors.setFixValuesDelegate(LAMBDA((), ->, void, {
        compassSensor.setFixValues(true);
    }));
    
#define sonicOperation(sonic, ope)              \
    do {                                        \
        switch (ope) {                          \
            case GCThreeSonicSensors::GetValue: \
                return sonic.getValue();        \
            case GCThreeSonicSensors::IsClose:  \
                return sonic.isClose();         \
            case GCThreeSonicSensors::IsFar:    \
                return sonic.isFar();           \
        }                                       \
        return 0;                               \
    } while (0)
    
    sonicSensors.setRightSonicDelegate(LAMBDA((GCThreeSonicSensors::DelegateOperation operation), ->, unsigned long, {
        sonicOperation(rightSonicSensor, operation);
    }));
    sonicSensors.setLeftSonicDelegate(LAMBDA((GCThreeSonicSensors::DelegateOperation operation), ->, unsigned long, {
        sonicOperation(leftSonicSensor, operation);
    }));
    sonicSensors.setBackSonicDelegate(LAMBDA((GCThreeSonicSensors::DelegateOperation operation), ->, unsigned long, {
        sonicOperation(backSonicSensor, operation);
    }));
    
#undef sonicOperation
    
    compassSensor.setRightFixValueDelegate(LAMBDA((int fixValue), ->, void, {
        rightMotor.fixPower = fixValue;
    }));
    compassSensor.setLeftFixValueDelegate(LAMBDA((int fixValue), ->, void, {
        leftMotor.fixPower = fixValue;
    }));
    compassSensor.setBackFixValueDelegate(LAMBDA((int fixValue), ->, void, {
        backMotor.fixPower = fixValue;
    }));
    
#ifdef ARDUINO
    /* Power Saving Configurations */
    power_all_disable();
    power_twi_enable();
    power_adc_enable();
    if (Serial) power_usart0_enable();
    power_timer0_enable();
    power_timer3_enable();
    power_timer4_enable();
    /* Disable analog comparator */
    ACSR |= 1 << ACD;
    
    /* Watchdog Timer Configurations */
    GCWatchdogTimer::init();
#endif
}

void AppDelegate::powerDownCPU()
{
#ifdef ARDUINO
    compassSensor.enterSleep();
    cli();
    GCWatchdogTimer::setDuration(GCWatchdogTimer::Duration_250);
    GCWatchdogTimer::enableInterrupt();
    power_twi_disable();
    power_adc_disable();
    sei();
    
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    
    cli();
    GCWatchdogTimer::disableInterrupt();
    power_twi_enable();
    power_adc_enable();
    sei();
    compassSensor.exitSleep();
#endif
}

void AppDelegate::idleCPU()
{
#ifdef ARDUINO
    compassSensor.enterSleep();
    cli();
    GCWatchdogTimer::enableInterrupt();
    power_twi_disable();
    power_adc_disable();
    sei();
    
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
    
    cli();
    GCWatchdogTimer::disableInterrupt();
    power_twi_enable();
    power_adc_enable();
    sei();
    compassSensor.exitSleep();
#endif
}

#ifdef ARDUINO
__attribute__((noreturn))
#endif
void AppDelegate::start()
{
    while (1) {
        if (rightSwitch) {
#if defined(PRINT_IR) && defined(ARDUINO)
            GCIRSensor::getValues(irSensors, numOfIRSensors);
            if (irSensors->value) irSensors->value += 8;
            for (int i = 0; i < numOfIRSensors; i++) {
                Serial.print(irSensors[i].value); Serial.print(F(","));
                irSensors[i].value = 0;
            }
            Serial.print('\n');
            Serial.flush();
#elif defined(LOGGING) && defined(ARDUINO)
            /* do what you want with arduino */
            printf_P(
                PSTR("%d\t%d\n%d\t%d\n\n"),
                leftFrontLineSensor.getValue(), rightFrontLineSensor.getValue(),
                leftBackLineSensor.getValue(), rightBackLineSensor.getValue()
            );
#elif defined(LOGGING)
            /* do what you want on OS X */
#elif defined(DEBUGGING)
            /* other debugging */
#else
            GCIRSensor::getValues(irSensors, numOfIRSensors);
            constexpr int frontIndex = 0;
            if (abs(lastTilt) <= GCBallDegrees337) {
                sonicSensors.getValues();
            }
            if (irSensors[frontIndex].value) {
                GCIRSensor::getMedian(irSensors, numOfIRSensors);
                motors.move(GCBallDegrees0, GCIRSensor::ballDistance, (lastTilt = compassSensor.setFixValues(true)), ! leftSwitch);
            } else {
                GCBallDegrees median = GCIRSensor::getMedian(irSensors, numOfIRSensors);
                if (median != GCBallDegreesNone) {
                    motors.move(median, GCIRSensor::ballDistance, (lastTilt = compassSensor.setFixValues(true)), ! leftSwitch);
                } else {
                    motors.move(GCBallDegreesNone, GCIRSensor::ballDistance, (lastTilt = compassSensor.setFixValues(false)), ! leftSwitch);
                    if (rightMotor.getCurrentSpeed() == 0 && leftMotor.getCurrentSpeed() == 0 && backMotor.getCurrentSpeed() == 0) {
                        idleCPU();
                    }
                }
            }
#endif

#if !defined(PRINT_IR) && !defined(LOGGING) && !defined(DEBUGGING)
        } else if (leftSwitch) {
            rightMotor.fixPower = 0;
            leftMotor.fixPower  = 0;
            backMotor.fixPower  = 0;
            constexpr int calibrationPower = 10;
            rightMotor.setPower(calibrationPower);
            leftMotor.setPower(-calibrationPower);
            backMotor.setPower(calibrationPower);
            delay(1000);
            
            compassSensor.calibration();
            
            rightMotor.freeImmediately();
            leftMotor.freeImmediately();
            backMotor.freeImmediately();
            
            while (leftSwitch) powerDownCPU();
        } else {
            if (compassSwitch) compassSensor.setOffset();
            rightMotor.freeImmediately();
            leftMotor.freeImmediately();
            backMotor.freeImmediately();
            sonicSensors.validateAll();
            lastTilt = compassSensor.setFixValues(false);
            powerDownCPU();
#elif defined(ARDUINO)
        } else if (leftSwitch) {
            rightMotor.fixPower = 0;
            leftMotor.fixPower  = 0;
            backMotor.fixPower  = 0;
            constexpr int calibrationPower = 10;
            rightMotor.freeImmediately();
            leftMotor.freeImmediately();
            backMotor.setPower(calibrationPower);
            
            compassSensor.calibration();
            
            backMotor.freeImmediately();
            
            while (leftSwitch) powerDownCPU();
        } else {
            rightMotor.freeImmediately();
            leftMotor.freeImmediately();
            backMotor.freeImmediately();
            compassSensor.setOffset();
#endif
        }
        
#ifndef ARDUINO
        return;
#endif
    }
}

#ifdef PRINT_IR
#undef PRINT_IR
#endif

#ifdef LOGGING
#undef LOGGING
#endif

#ifdef DEBUG
#undef DEBUG
#endif
