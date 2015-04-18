#ifndef Japan2015_GCArduino_h
#define Japan2015_GCArduino_h

#ifdef ARDUINO

#include <Arduino.h>

#else

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#undef F
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(string_literal))

#ifdef __cplusplus
extern "C"{
#endif
    
#define HIGH 0x1
#define LOW  0x0
    
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
    
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
    
#define SERIAL  0x0
#define DISPLAY 0x1
    
#define LSBFIRST 0
#define MSBFIRST 1
    
#define CHANGE 1
#define FALLING 2
#define RISING 3
    
// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
    
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))
    
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))
    
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
    
    
typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef uint8_t boolean;
typedef uint8_t byte;

void init(void);

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
    
#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#endif
    
#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER2  5
#define TIMER2A 6
#define TIMER2B 7
    
#define TIMER3A 8
#define TIMER3B 9
#define TIMER3C 10
#define TIMER4A 11
#define TIMER4B 12
#define TIMER4C 13
#define TIMER4D 14
#define TIMER5A 15
#define TIMER5B 16
#define TIMER5C 17
    
#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
class __FlashStringHelper;

#endif // ifdef __cplusplus

#endif // ifdef ARDUINO

#endif // #ifndef Japan2015_GCArduino_h
