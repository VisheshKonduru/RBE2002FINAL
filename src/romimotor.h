// romimotor.h
#pragma once

#include <Arduino.h>

/** \file Controls ESCs on pins 9 and 10 using Timer1.
 *
 * Pin 9 uses OCR1A (Timer1 Channel A)
 * Pin 10 uses OCR1B (Timer1 Channel B)
 *
 * The 16-bit Timer1 is set up with a prescaler and TOP value to achieve a 50Hz PWM frequency for servos.
 */

class RomiServoBase
{
protected:
    uint16_t usMin = 1000;
    uint16_t usMax = 2000;
    bool isAttached = false;

public:
    virtual void attach(void) = 0;
    virtual void detach(void) = 0;

    uint16_t setMinMaxMicroseconds(uint16_t min, uint16_t max)
    {
        if (min > max)
        {
            uint16_t temp = min;
            min = max;
            max = temp;
        }
        usMin = min;
        usMax = max;
        return usMax - usMin;
    }

    virtual void writeMicroseconds(uint16_t microseconds) = 0;
};

class RomiServoPin9 : public RomiServoBase
{
public:
    void attach(void);
    void detach(void);

protected:
    void writeMicroseconds(uint16_t microseconds);
};

class RomiServoPin10 : public RomiServoBase
{
public:
    void attach(void);
    void detach(void);

protected:
    void writeMicroseconds(uint16_t microseconds);
};