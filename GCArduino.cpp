#ifndef ARDUINO

#include "GCArduino.h"
#include <stdio.h>
#include <unistd.h>

void init()
{
    
}

void pinMode(uint8_t pin1, uint8_t pin2)
{
    printf("pinMode(%d, %d)\n", pin1, pin2);
}

void digitalWrite(uint8_t pin, uint8_t value)
{
    printf("digitalWrite(%d, %d)\n", pin, value);
}

int digitalRead(uint8_t pin)
{
    printf("digitalRead(%d)\n", pin);
    return 0;
}

int analogRead(uint8_t pin)
{
    printf("analogRead(%d)\n", pin);
    return 0;
}

void analogReference(uint8_t __attribute__((unused)) mode)
{
    
}

void analogWrite(uint8_t pin, int value)
{
    printf("analogWrite(%d, %d);\n", pin, value);
}

unsigned long millis(void)
{
    unsigned long time = 0; // edit with debugger
    return time;
}

unsigned long micros(void)
{
    unsigned long time = 0; // edit with debugger
    return time;
}

void delay(unsigned long ms)
{
    usleep(static_cast<useconds_t>(ms / 1000));
}

void delayMicroseconds(unsigned int us)
{
    usleep(us);
}

unsigned long pulseIn(uint8_t __attribute__((unused)) pin, uint8_t __attribute__((unused)) state, unsigned long __attribute__((unused)) timeout)
{
    unsigned long time = 0;
    return time;
}

#endif
