#include "LineSensor.h"

#define DARK_THRESHOLD 500;

void LineSensor::Initialize(void)
{
    pinMode(leftSensorPin, INPUT);
    pinMode(secondSensorPin, INPUT);
    pinMode(thirdSensorPin, INPUT);
    pinMode(fourthSensorPin, INPUT);
    pinMode(fifthSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
}

float LineSensor::CalcError(void) 
{ 
    // return analogRead(leftSensorPin) - analogRead(secondSensorPin); 
    const int sensorPins[6] = {leftSensorPin, secondSensorPin, thirdSensorPin, fourthSensorPin, fifthSensorPin, rightSensorPin};
    // const float weights[6] = {-1.5, -1.0, -0.5, 0.5, 1, 1.5};
    const int weights[6] = {-3, -2, -1, 1, 2, 3}; 
    float weightedSum = 0, totalSum = 0;

    // for (int i = 0; i < 2; i++) {
    //     sensorValues[i] = analogRead(sensorPins[i]);
    //     weightedSum += (sensorValues[i] - sensorValues[5-i]) * weights[i];
    // }

    // return totalSum == 0 ? 0 : (weightedSum / totalSum) * 100;
    // return weightedSum / 10;

    for (int i = 0; i < 6; i++) {
        float sensorValue = analogRead(sensorPins[i]);
        weightedSum += sensorValue * weights[i];
        totalSum += sensorValue;
    }

    if (totalSum == 0) {return 0;}
    return (weightedSum / totalSum);
}
    

bool LineSensor::CheckIntersection(void)
{
    bool retVal = false;

    // bool isLeftDark = analogRead(leftSensorPin) > DARK_THRESHOLD;
    // bool isRightDark = analogRead(rightSensorPin) > DARK_THRESHOLD;

    // bool onIntersection = isLeftDark && isRightDark;
    // if(onIntersection && !prevOnIntersection) retVal = true;

    // prevOnIntersection = onIntersection;

    return retVal;
}