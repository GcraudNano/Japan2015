#ifndef __GCRobot__
#define __GCRobot__

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wunused"
#pragma GCC diagnostic ignored "-Wcast-qual"

#define __STDC_LIMIT_MACROS
#include "GCArduino.h"
#undef __STDC_LIMIT_MACROS
#include <stdio.h>

#ifdef ARDUINO
#include <wiring_private.h>
#endif

#pragma GCC diagnostic pop
#undef round
#define round(x) (static_cast<long>(x)>=0?static_cast<long>((x)+0.5):static_cast<long>((x)-0.5))

#define INLINE inline __attribute__((always_inline))

#define LAMBDA(arglist, arrow, retType, body)	\
({                                              \
    class Lambda                                \
    {                                           \
    public:                                     \
        static retType func arglist body        \
    };                                          \
    Lambda::func;                               \
})

//#define NO_DYING_MESSAGE

static inline constexpr double safeRadians(int degrees)
{
    return degrees > 90 && degrees <= 180 ? radians(180 - degrees) :
    degrees > 180 && degrees <= 270 ? radians(180 - degrees) :
    degrees > 270 ? safeRadians(degrees - 360) :
    degrees < -90 ? safeRadians(degrees + 360) :
    radians(degrees);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Weffc++"

class GCRobot {
    static constexpr int eepromStatingPoint = 0;
    static void __attribute__((constructor)) initialize();
    
public:
#if ! (defined(ARDUINO) && ! defined(NO_DYING_MESSAGE))
    __attribute__((noreturn))
#endif
    static void die(const __FlashStringHelper *fmt, ...);
};

class DigitalPin {
protected:
#ifndef ARDUINO
    uint8_t pin;
#endif
    uint8_t bit;
    uint8_t port;
    virtual ~DigitalPin() {}
    
public:
    DigitalPin(uint8_t pin);
};

class DigitalIn : public DigitalPin {
protected:
    volatile uint8_t *reg;
    
public:
    DigitalIn(uint8_t pin);
    DigitalIn(uint8_t pin, uint8_t mode);
    virtual ~DigitalIn() {}
    INLINE virtual int read() {
#ifdef ARDUINO
        return (*reg & bit) ? HIGH : LOW;
#else
        return digitalRead(pin);
#endif
    }
    INLINE virtual operator int() {
        return read();
    }
    INLINE virtual unsigned long readPulse(uint8_t state, unsigned long timeout = 100000L) {
#ifdef ARDUINO
        uint8_t stateMask = (state ? bit : 0);
        unsigned long width = 0;
        unsigned long numOfLoops = 0;
        unsigned long maxLoops = microsecondsToClockCycles(timeout) / 16;
        
        while ((*reg & bit) == stateMask)
            if (numOfLoops++ == maxLoops)
                return 0;
        
        while ((*reg & bit) != stateMask)
            if (numOfLoops++ == maxLoops)
                return 0;
        
        while ((*reg & bit) == stateMask) {
            if (numOfLoops++ == maxLoops)
                return 0;
            width++;
        }
        
        return clockCyclesToMicroseconds(width * 21 + 16);
#else
        return pulseIn(pin, state, timeout);
#endif
    }
};

#ifdef __cplusplus
extern "C" {
#endif
    void turnOffPWM(uint8_t timer);
#ifdef __cplusplus
}
#endif

class DigitalInWithPWM : public DigitalIn {
    uint8_t timer;
    
public:
    DigitalInWithPWM(uint8_t pin);
    ~DigitalInWithPWM() {}
    INLINE int read() {
#ifdef ARDUINO
        turnOffPWM(timer);
#endif
        return DigitalIn::read();
    }
    INLINE operator int() {
        return read();
    }
    INLINE unsigned long readPulse(uint8_t state, unsigned long timeout = 100000L) {
#ifdef ARDUINO
        turnOffPWM(timer);
#endif
        return DigitalIn::readPulse(state, timeout);
    }
};

class DigitalOut : public DigitalPin {
protected:
    volatile uint8_t *out;
    
public:
    DigitalOut(uint8_t pin);
    virtual ~DigitalOut() {}
    INLINE virtual void write(int value) {
#ifdef ARDUINO
        uint8_t oldSREG = SREG;
        cli();
        
        if (value == LOW) {
            *out &= ~bit;
        } else {
            *out |= bit;
        }
        
        SREG = oldSREG;
#else
        digitalWrite(pin, static_cast<uint8_t>(value));
#endif
    }
    INLINE DigitalOut& operator = (int value) {
        write(value);
        return *this;
    }
};

class DigitalOutWithPWM : public DigitalOut {
    uint8_t timer;
    
public:
    DigitalOutWithPWM(uint8_t pin);
    ~DigitalOutWithPWM() {}
    INLINE void write(int value) {
#ifdef ARDUINO
        turnOffPWM(timer);
#endif
        DigitalOut::write(value);
    }
    INLINE DigitalOutWithPWM& operator = (int value) {
        write(value);
        return *this;
    }
};

class DigitalInOut : public DigitalIn, public DigitalOut {
    volatile uint8_t *modeReg;
    
public:
    DigitalInOut(uint8_t pin);
    ~DigitalInOut() {}
    
    void setMode(uint8_t mode) {
#ifdef ARDUINO
        if (mode == INPUT) {
            uint8_t oldSREG = SREG;
            cli();
            *modeReg &= ~ DigitalIn::bit;
            *out     &= ~ DigitalIn::bit;
            SREG = oldSREG;
        } else if (mode == INPUT_PULLUP) {
            uint8_t oldSREG = SREG;
            cli();
            *modeReg &= ~ DigitalIn::bit;
            *out     |=   DigitalIn::bit;
            SREG = oldSREG;
        } else {
            uint8_t oldSREG = SREG;
            cli();
            *modeReg |= DigitalIn::bit;
            SREG = oldSREG;
        }
#else
        pinMode(DigitalIn::pin, mode);
#endif
    }
};

class AnalogIn {
    uint8_t pin;
    
public:
    AnalogIn(uint8_t pin);
    INLINE int read() {
        return analogRead(pin);
    }
    INLINE operator int() {
        return read();
    }
};

class AnalogOut {
    uint8_t pin;
    
public:
    AnalogOut(uint8_t pin);
    INLINE void write(int value) {
        analogWrite(pin, value);
    }
    INLINE AnalogOut& operator = (int value) {
        write(value);
        return *this;
    }
};

#pragma GCC diagnostic pop

#endif
