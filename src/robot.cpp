#include "robot.h"
#include <IRdecoder.h>


void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();

    /**
     * Initialize the IMU and set the rate and scale to reasonable values.
     */
    imu.init();

    /**
     * TODO: Add code to set the data rate and scale of IMU (or edit LSM6::setDefaults())
     */

    // The line sensor elements default to INPUTs, but we'll initialize anyways, for completeness
    lineSensor.Initialize();
}

void Robot::EnterIdleState(void)
{
    Serial.println("-> IDLE");
    chassis.Stop();
    keyString = "";
    robotState = ROBOT_IDLE;
}

int turns = 0;
/**
 * Functions related to the IMU (turning; ramp detection)
 */
void Robot::EnterTurn(int desiredDirection)
{
    Serial.println(" -> TURN");
    robotState = ROBOT_TURNING;

    // Set target direction and calculate turning angle
    targetDirection = desiredDirection;
    int turns = (targetDirection - currDirection + 4) % 4; // Normalize to [0, 3]

    if (turns == 1 || turns == -3)
    {
        chassis.SetTwist(0, 1); // Turn right
        targetHeading = eulerAngles.z + 90;
    }
    else if (turns == -1 || turns == 3)
    {
        chassis.SetTwist(0, -1); // Turn left
        targetHeading = eulerAngles.z - 90;
    }
    else if (turns == 2 || turns == -2)
    {
        chassis.SetTwist(0, 1); // Turn 180 degrees
        targetHeading = eulerAngles.z + 180;
    }

    Serial.print("Turning to direction: ");
    Serial.println(targetDirection);
}


bool Robot::CheckTurnComplete(void)
{

    /**
     * TODO: add a checker to detect when the turn is complete
     */
    if (targetDirection - currDirection > 0){
        if (eulerAngles.z > targetHeading){
        return true;
        Serial.println("checking");
        } 

    } else if (targetDirection - currDirection < 0){
        if(eulerAngles.z < targetHeading) {
            return true;
            Serial.println("checking");
        }

    } else {
        return false;
    }

}

void Robot::HandleTurnComplete(void)
{
    Serial.println("Turn complete");

    // Update current direction
    currDirection = targetDirection;

    // Stop and continue line following
    chassis.Stop();
    EnterLineFollowing(baseSpeed);
}


/**
 * Here is a good example of handling information differently, depending on the state.
 * If the Romi is not moving, we can update the bias (but be careful when you first start up!).
 * When it's moving, then we update the heading.
 */
void Robot::HandleOrientationUpdate(void)
{
    prevEulerAngles = eulerAngles;
    if(robotState == ROBOT_IDLE)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
    }

    else // update orientation
    {
        eulerAngles.z =  prevEulerAngles.z + ((imu.g.z - imu.gyroBias.z) * (1.0 / imu.gyroODR) * (imu.mdpsPerLSB / 1000));
        eulerAngles.y =  prevEulerAngles.y + ((imu.g.y - imu.gyroBias.y) * (1.0 / imu.gyroODR) * (imu.mdpsPerLSB / 1000));
        eulerAngles.x =  prevEulerAngles.x + ((imu.g.x - imu.gyroBias.x) * (1.0 / imu.gyroODR) * (imu.mdpsPerLSB / 1000));
        // Serial.println(eulerAngles.z);
    }

#ifdef __IMU_DEBUG__
    // Serial.print(imu.g.x);
    // Serial.print('\t');
    // Serial.print(imu.g.y);
    // Serial.print('\t');
    // Serial.print(imu.g.z);
    // Serial.print('\t');
    // Serial.print(millis());
    // Serial.print('\n');

    Serial.print(eulerAngles.x);
    Serial.print('\t');
    Serial.print(eulerAngles.y);
    Serial.print('\t');
    Serial.print(eulerAngles.z);
    Serial.print('\n');
#endif
}

/**
 * Functions related to line following and intersection detection.
 */
void Robot::EnterLineFollowing(float speed)
{
    Serial.println(" -> LINING");
    baseSpeed = speed;

    // Check if the heading needs adjustment
    if (currDirection == targetDirection)
    {
        robotState = ROBOT_LINING;
    }
    else
    {
        EnterTurn(currDirection); // Ensure alignment with the desired direction
    }
}





void Robot::LineFollowingUpdate(void)
{
    if (robotState == ROBOT_LINING)
    {
        // Calculate the error in CalcError(), calculate the effort, and update the motion
        float lineError = lineSensor.CalcError();

        float turnEffort = 1 * ((lineError * 0.5) + (0.1 * (lineError - prevLineError)));
        prevLineError = lineError;

        // Maintain base speed during line following
        chassis.SetTwist(baseSpeed, turnEffort);
    }

    if (robotState == ROBOT_CORRECTING)
    {
        HandleCorrecting();
    }
}



/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */
void Robot::HandleIntersection(void)
{
    if (robotState == ROBOT_LINING)
    {
        Serial.println("Intersection detected");

        // Update the grid based on current direction
        switch (currDirection)
        {
            case 0: iGrid++; break; // EAST
            case 1: jGrid++; break; // NORTH
            case 2: iGrid--; break; // WEST
            case 3: jGrid--; break; // SOUTH
        }

        // Debugging: Print current position
        Serial.print("Current Position: (");
        Serial.print(iGrid);
        Serial.print(", ");
        Serial.print(jGrid);
        Serial.println(")");

        // Check if target is reached
        if (iGrid == iTarget && jGrid == jTarget)
        {
            EnterIdleState(); // Stop when both targets are reached
        }
        else
        {
            // Prioritize `jGrid` adjustments first
            if (jGrid != jTarget)
            {
                int desiredDirection = (jGrid < jTarget) ? 1 : 3; // NORTH or SOUTH
                if (currDirection != desiredDirection)
                {
                    EnterTurn(desiredDirection); // Turn to correct direction
                }
                else
                {
                    EnterCorrecting(500); // Center on intersection
                }
            }
            else if (iGrid != iTarget) // Now adjust `iGrid`
            {
                int desiredDirection = (iGrid < iTarget) ? 0 : 2; // EAST or WEST
                if (currDirection != desiredDirection)
                {
                    EnterTurn(desiredDirection); // Turn to correct direction
                }
                else
                {
                    EnterCorrecting(500); // Center on intersection
                }
            }
        }
    }
}








void Robot::EnterCorrecting(int time)
{
    Serial.println(" -> CORRECTING");
    robotState = ROBOT_CORRECTING;
    targetTime = millis() + time;
    chassis.SetWheelSpeeds(10, 10); // Slow forward motion for centering
}

void Robot::HandleCorrecting(void)
{
    if (millis() < targetTime)
    {
        chassis.SetWheelSpeeds(10, 10); // Keep moving forward
    }
    else
    {
        chassis.Stop(); // Stop once centering is complete
        EnterLineFollowing(baseSpeed); // Transition to line following
    }
}




void Robot::RobotLoop(void) 
{
    /**
     * The main loop for your robot. Process both synchronous events (motor control),
     * and asynchronous events (IR presses, distance readings, etc.).
    */

    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);
    
//  //Serial.println(lineSensor.CheckIntersection
    /// erial.println(analogRead(A6));
    /// erial.println(analogRead(A0));());

    /**
     * Check the Chassis timer, which is used for executing motor control
     */
    if(chassis.CheckChassisTimer())
    {
        // add synchronous, pre-motor-update actions here
        if(robotState == ROBOT_CORRECTING){
            
            HandleCorrecting();
        }
        if(robotState == ROBOT_LINING)
        {
            LineFollowingUpdate();
        }

        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }

    /**
     * Check for any intersections
     */
    if(lineSensor.CheckIntersection()){
        HandleIntersection();
    } 


    /**
     * Check for an IMU update
     */
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
        if(CheckTurnComplete()) HandleTurnComplete();

    }

}
