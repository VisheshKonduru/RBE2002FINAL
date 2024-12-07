#include "robot.h"

#include <Arduino.h>
#include <Wire.h>
#include <OpenMV.h>


#define CAMERA_ADDRESS 0x12 // I2C address of the OpenMV camera
#define SDA_PIN 2
#define SCL_PIN 3


void Robot::Setup(){
    Wire.begin();
    camera.begin();


}

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

void Robot::ApproachTag() {
    // Check if any AprilTags are detected
    uint8_t tagCount = camera.getTagCount();
    if (tagCount > 0) {
        AprilTagDatum tag;
        if (camera.readTag(tag)) {
            // Extract tag information
            int tagID = tag.id;
            int centerX = tag.cx;
            int centerY = tag.cy;

            // Desired position is the center of the camera frame
            const int desiredX = 80; // Adjust based on camera resolution

            // Calculate the error in the X-axis
            int errorX = centerX - desiredX;

            // Calculate the twist using proportional control
            float twist = approachKp * errorX;

            // Set a constant forward speed
            float forwardSpeed = -7; // Adjust as needed

            // Update the chassis motion
            chassis.SetTwist(forwardSpeed, twist);

            // Optional: Print debug information
            // Serial.print("Approaching tag ID ");
            // Serial.print(tagID);
            // Serial.print(": Forward Speed = ");
            // Serial.print(forwardSpeed);
            // Serial.print(", Twist = ");
            // Serial.println(twist);
        } else {
            // Failed to read tag data
            chassis.SetTwist(0, 0);
            EnterSearchingState();
        }
    } else {
        // No tags detected
        chassis.SetTwist(0, 0);
        EnterSearchingState();
    }
}





