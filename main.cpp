#include "AppDelegate.h"

#ifdef ARDUINO
void __attribute__((section(".init5"))) __attribute__((naked)) pullUpEveryPin()
{
    /* The datasheet says pulling up every pin which is unused is good for the power performance */
    for (int i = 0; i < NUM_DIGITAL_PINS; i++) {
        pinMode(i, INPUT_PULLUP);
    }
}
#endif

int main(int __attribute__((unused)) argc, const char __attribute__((unused)) * argv[], const char __attribute__((unused)) * evnp[])
{
    AppDelegate().start();

    return 0;
}
