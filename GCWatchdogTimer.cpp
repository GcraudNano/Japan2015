#include "GCWatchdogTimer.h"
#ifdef ARDUINO
#include <avr/wdt.h>
#endif

void GCWatchdogTimer::init()
{
#ifdef ARDUINO
    cli();
    wdt_reset();
    MCUSR &= ~(1 << WDRF);
    WDTCSR = 1 << WDCE;
    sei();
#endif
}

#ifdef ARDUINO
void GCWatchdogTimer::setDuration(GCWatchdogTimerDuration duration)
#else
void GCWatchdogTimer::setDuration(GCWatchdogTimerDuration __attribute__((unused)) duration)
#endif
{
#ifdef ARDUINO
    cli();
    WDTCSR |= duration;
    sei();
#endif
}

void GCWatchdogTimer::enableInterrupt()
{
#ifdef ARDUINO
    cli();
    WDTCSR |= 1 << WDIE;
    sei();
#endif
}

void GCWatchdogTimer::disableInterrupt()
{
#ifdef ARDUINO
    cli();
    WDTCSR &= ~(1 << WDIE);
    sei();
#endif
}

void GCWatchdogTimer::enableReset()
{
#ifdef ARDUINO
    cli();
    WDTCSR |= 1 << WDE;
    sei();
#endif
}

void GCWatchdogTimer::disableReset()
{
#ifdef ARDUINO
    cli();
    WDTCSR &= ~(1 << WDE);
    sei();
#endif
}

#ifdef ARDUINO
ISR(WDT_vect)
{
}
#endif
