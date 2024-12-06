#include "robot.h"
#include "OpenMV.h"
#include <Arduino.h>


void Robot::HandleAprilTag(const AprilTagDatum& tag)
{
    // Serial.print("Tag: ");
    // Serial.print(tag. id); Serial.print('\t');
    // Serial.print(tag.cx); Serial.print('\t');
    // Serial.print(tag.cy); Serial.print('\t');
    // Serial.print(tag.h); Serial.print('\t');
    // Serial.print(tag.w); Serial.print('\t');
    // Serial.print(tag.rot); //rotated angle; try turning the tag
    // Serial.print('\n');
    

    //** TODO: Add code to hangle a tag in APPROACHING and SEARCHING states */
    currentTag = tag;
    tagDetected = true;
    tagID = tag.id;    
    detectedTagID = tag.id; // assigning the detected tag ID to our global variable for non-blocking blinking
    //storing 


    // Handling behavior based on state
    if(robotState == ROBOT_SEARCHING)
    {
        EnterApproachingState();
        Serial.println("Tag Detected, transitioning to Approaching State");
    }
    else if (robotState == ROBOT_APPROACHING)
    {
        
        Serial.println("updating parameeters");
    }
    tagLossTimer.Start(tagLossTimeout); //starts timer
}


// defining our  LED pin
const int LED_PIN = 13;

// our varibles for non-blocking blinking
unsigned long previousMillis = 0;
const long interval = 500; // 500ms interval
int blinkCount = 0;
int tagID = 0; // variable to store the tag ID


void Robot::EnterSearchingState(void)
{
    /** TODO Set romi to slowly spin to look for tags */
    robotState = ROBOT_SEARCHING;

    //set motors to slowly spin
    float searchTwistSpeed = 0.2;
    chassis.SetTwist(0, searchTwistSpeed); // should be 0.2 rad/s

    //stops tag loss timer 
    tagLossTimer.Cancel();


}

void Robot::EnterApproachingState(void)
{
    /** TODO  turn on the LED when Romi finds a tag. 
     * blink out a number that corresponds to the tag ID (has to be non-blocking)
     * Be sure to add code elsewhere to turn off the LED when ROMi 
     * done aligning
    */
   //set the robot state to approaching
   robotState = ROBOT_APPROACHING;

    //turn on the LED
    digitalWrite(LED_PIN, HIGH);

    //Initalize the blink count and our tag ID
    blinkCount = 0;
    tagID = detectedTagID; // assumes the dectectedTagID is set when tag is found
   Serial.print("going into approaching state and aligning with tag:");
}

/** Note that the tolerances are integers, since camera works 
 * in tnteger pixels, if you have another method for calculations
 * you may need floats
 */
bool Robot::CheckApproachComplete(int headingTolerance, int distanceTolerance)
{
    /** TODO add code to determine if the robot is at the correct location */
    const int tagSizeThreshold = 4000; // threshold for tag size
    bool isCloseEnough = (currentTag.h * currentTag.w) > tagSizeThreshold;
    // Non-blocking blink logic 

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Toggle the LED
        if (blinkCount < tagID *2) {// blinks twice for each count
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            blinkCount++;
            } else {
                // ensuring our LED stays on after blinking the tag ID
                digitalWrite(LED_PIN, HIGH);
            }
    }
    return isCloseEnough;
}

void Robot::ApproachTag(void)
{
    if(!tagDetected)
    {
        //not tage detected, robot stops and goes back to searching 
        chassis.SetTwist(0, 0);

        //trasition to our searching state
        EnterSearchingState();
        return;
    }

    //Desired Position is center to camera Fram
    const int desiredX = 80;

    // Calculate the error in the X-Axis
    int errorX = currentTag.cx - desiredX;

    // Calculate the twist using Prop control
    float twist = approachKp * errorX; // neg to correct direction towards tag

    // Limit the twist to max NOT FUCKING WORKING
    //if(twist > maxApproachSpeed) twist = maxApproachSpeed;
    //else if(twist < -maxApproachSpeed) twist = -maxApproachSpeed;

    // set const forward speed 
    float forwardSpeed = -7;

    //Update the chassis motion
    chassis.SetTwist(forwardSpeed, twist);

    Serial.print("Approaching: Forward Speed =");
    Serial.print(forwardSpeed);
    Serial.print(", Twist =");
    Serial.print(twist);

    chassis.UpdateMotors();

    // check if our robot is close enough to tag
    if (CheckApproachComplete(20, 5000))
{
    // Stop our robot
    chassis.SetTwist(0, 0);
    chassis.UpdateMotors();


    // Transition to IDLE 
    EnterIdleState();

    Serial.println("Approach Complete");
}
}





