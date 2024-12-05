// romimotor.cpp
#include "romimotor.h"

void RomiServoPin9::attach(void)
{
    pinMode(9, OUTPUT); // Set pin 9 as OUTPUT

    cli(); // Disable interrupts

    // Configure Timer1 for Fast PWM mode 14, with ICR1 as TOP
    TCCR1A = _BV(WGM11) | _BV(COM1A1);             // Mode 14 (Fast PWM), Clear OC1A on Compare Match
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // Prescaler = 8
    ICR1 = 39999; // 20 ms period (50Hz PWM frequency)

    sei(); // Enable interrupts

    isAttached = true;
}

void RomiServoPin9::detach(void)
{
    cli();

    // Disable PWM on OC1A (pin 9)
    TCCR1A &= ~_BV(COM1A1);

    sei();

    isAttached = false;
}

void RomiServoPin9::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    // For 16MHz clock and prescaler of 8, each timer tick is 0.5 us
    uint16_t dutyCycle = microseconds * 2; // Convert microseconds to timer ticks
    OCR1A = dutyCycle;
}

void RomiServoPin10::attach(void)
{
    pinMode(10, OUTPUT); // Set pin 10 as OUTPUT

    cli();

    // Configure Timer1 for Fast PWM mode 14, with ICR1 as TOP
    TCCR1A |= _BV(COM1B1); // Enable PWM on OC1B
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // Prescaler = 8
    ICR1 = 39999; // 20 ms period (50Hz PWM frequency)

    sei();

    isAttached = true;
}

void RomiServoPin10::detach(void)
{
    cli();

    // Disable PWM on OC1B (pin 10)
    TCCR1A &= ~_BV(COM1B1);

    sei();

    isAttached = false;
}

void RomiServoPin10::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    // For 16MHz clock and prescaler of 8, each timer tick is 0.5 us
    uint16_t dutyCycle = microseconds * 2; // Convert microseconds to timer ticks
    OCR1B = dutyCycle;
}