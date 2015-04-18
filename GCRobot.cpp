#include "GCRobot.h"
#ifdef ARDUINO

#include <wiring_private.h>
#include <avr/eeprom.h>

#endif

#undef portOutputRegister
#undef portInputRegister
#undef portModeRegister
#define portOutputRegister(P) ( reinterpret_cast<volatile uint8_t *>( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( reinterpret_cast<volatile uint8_t *>( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( reinterpret_cast<volatile uint8_t *>( pgm_read_word( port_to_mode_PGM + (P))) )

void /*__attribute__((constructor))*/ GCRobot::initialize()
{
#ifdef ARDUINO
    init();
#   if defined(USBCON)
    USBDevice.attach();
#   endif
    
    char c = eeprom_read_byte(reinterpret_cast<unsigned char *>(eepromStatingPoint));
    if (c != -1) {
        int address = eepromStatingPoint;
        Serial.begin(9600);
        while (c && c != -1) {
            Serial.print(c);
            c = eeprom_read_byte(reinterpret_cast<unsigned char *>(++address));
        }
        Serial.println();
        Serial.print(F("Press any key to clear..."));
        while (1) {
            if (Serial.available()) {
                Serial.println();
                Serial.print(F("Cleaning"));
                for (int i = eepromStatingPoint; i < address; i++) {
                    eeprom_write_byte(reinterpret_cast<unsigned char *>(i), -1);
                    Serial.print(F("."));
                }
                Serial.println();
                Serial.println(F("Done!"));
                Serial.println(F("Fix the bug and reboot"));
                break;
            }
        }
        while (1) Serial.write(0);
    }
    
    static FILE out;
    fdev_setup_stream(&out, LAMBDA((char c, FILE __attribute__((unused)) *stream), ->, int, {
        if (c == '\n') Serial.print('\r');
        return Serial.print(c);
    }), NULL, _FDEV_SETUP_WRITE);
    stdout = &out;
#endif
}

#ifdef ARDUINO
#if defined(ARDUINO) && ! defined(NO_DYING_MESSAGE)
__attribute__((noreturn))
#endif
void GCRobot::die(const __FlashStringHelper *fmt, ...)
#else
void __attribute__((noreturn)) GCRobot::die(const __FlashStringHelper __attribute__((unused)) *fmt, ...)
#endif
{
#if defined(ARDUINO) && ! defined(NO_DYING_MESSAGE)
    Serial.begin(9600);
    FILE dummy;
    va_list list;
    va_start(list, fmt);
    static int address = eepromStatingPoint;
    fdev_setup_stream(&dummy, LAMBDA((char c, FILE __attribute__((unused)) *stream), ->, int, {
        size_t ret = Serial.print(c);
        eeprom_write_byte(reinterpret_cast<unsigned char *>(address++), c);
        return ret;
    }), NULL, _FDEV_SETUP_WRITE);
    vfprintf_P(&dummy, reinterpret_cast<const char *>(fmt), list);
    va_end(list);
    Serial.println();
    while (1) Serial.write(0);
#elif defined(ARDUINO)
    /* Just Ignore */
#else
    abort();
#endif
}

#undef bit

#ifdef ARDUINO
DigitalPin::DigitalPin(uint8_t pin) : bit(digitalPinToBitMask(pin)), port(digitalPinToPort(pin))
{
    if (port == NOT_A_PIN) {
        GCRobot::die(F("DigitalPin::DigitalPin() Invalid pin %d specified"), pin);
    }
}
#else
DigitalPin::DigitalPin(uint8_t _pin) : pin(_pin)
{
    
}
#endif


DigitalIn::DigitalIn(uint8_t pin) : DigitalIn(pin, INPUT_PULLUP)
{
}

#ifdef ARDUINO
DigitalIn::DigitalIn(uint8_t pin, uint8_t mode) : DigitalPin(pin), reg(portInputRegister(port))
{
    pinMode(pin, mode);
}
#else
DigitalIn::DigitalIn(uint8_t _pin, uint8_t _mode) : DigitalPin(_pin)
{
    pinMode(pin, _mode);
}
#endif

#ifdef ARDUINO
extern "C" {
    void turnOffPWM(uint8_t timer)
    {
#ifdef ARDUINO
        switch (timer)
        {
#if defined(TCCR1A) && defined(COM1A1)
            case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
#endif
#if defined(TCCR1A) && defined(COM1B1)
            case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
#endif
                
#if defined(TCCR2) && defined(COM21)
            case  TIMER2:   cbi(TCCR2, COM21);      break;
#endif
                
#if defined(TCCR0A) && defined(COM0A1)
            case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
#endif
                
#if defined(TIMER0B) && defined(COM0B1)
            case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
#endif
#if defined(TCCR2A) && defined(COM2A1)
            case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
#endif
#if defined(TCCR2A) && defined(COM2B1)
            case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
#endif
                
#if defined(TCCR3A) && defined(COM3A1)
            case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
#endif
#if defined(TCCR3A) && defined(COM3B1)
            case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
#endif
#if defined(TCCR3A) && defined(COM3C1)
            case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
#endif
                
#if defined(TCCR4A) && defined(COM4A1)
            case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
#endif
#if defined(TCCR4A) && defined(COM4B1)
            case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
#endif
#if defined(TCCR4A) && defined(COM4C1)
            case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
#endif
#if defined(TCCR4C) && defined(COM4D1)
            case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
#endif
                
#if defined(TCCR5A)
            case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
            case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
            case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
#endif
        }
#endif
    }
}
#endif

#ifdef ARDUINO
DigitalInWithPWM::DigitalInWithPWM(uint8_t pin) : DigitalIn(pin), timer(digitalPinToTimer(pin))
{
    if (timer == NOT_ON_TIMER) {
        GCRobot::die(F("DigitalInWithPWM::DigitalInWithPWM() Pin %d is not on timer"), pin);
    }
}
#else
DigitalInWithPWM::DigitalInWithPWM(uint8_t pin) : DigitalIn(pin)
{
    
}
#endif


// --------------------------------------

#ifdef ARDUINO
DigitalOut::DigitalOut(uint8_t pin) : DigitalPin(pin), out(portOutputRegister(port))
{
    pinMode(pin, OUTPUT);
}
#else
DigitalOut::DigitalOut(uint8_t pin) : DigitalPin(pin)
{
    
}
#endif

#ifdef ARDUINO
DigitalOutWithPWM::DigitalOutWithPWM(uint8_t pin) : DigitalOut(pin), timer(digitalPinToTimer(pin))
{
    if (timer == NOT_ON_TIMER) {
        GCRobot::die(F("DigitalOutWithPWM::DigitalOutWithPWM() Pin %d is not on timer"), pin);
    }
}
#else
DigitalOutWithPWM::DigitalOutWithPWM(uint8_t pin) : DigitalOut(pin)
{
    
}
#endif

#ifdef ARDUINO
DigitalInOut::DigitalInOut(uint8_t pin) : DigitalIn(pin), DigitalOut(pin), modeReg(portModeRegister(DigitalIn::port))
{
    
}
#else
DigitalInOut::DigitalInOut(uint8_t pin) : DigitalIn(pin), DigitalOut(pin)
{
    
}
#endif


// --------------------------------------

AnalogIn::AnalogIn(uint8_t _pin) : pin(_pin)
{
    pinMode(pin, INPUT);
}

// --------------------------------------

AnalogOut::AnalogOut(uint8_t _pin) : pin(_pin)
{
    pinMode(pin, OUTPUT);
}
