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

    /**
     * TODO: Add code to initiate the turn and set the target
     */
    targetDirection = desiredDirection;
    turns =  (targetDirection - currDirection);

    
    if (turns >  0){
        chassis.SetTwist(0,1);
        targetHeading = eulerAngles.z + (abs(turns) * 90);
    } else {
        chassis.SetTwist(0,-1);
        targetHeading = eulerAngles.z - (abs(turns) * 90);
    }
    
    Serial.println(targetHeading);
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
    /**
     * TODO: Add code to handle the completed turn
     * update direction based on the number of turns made
     */
    Serial.println("turn completed");
    currDirection = targetDirection;
    turns = 0;
    Serial.println(currDirection);
    chassis.Stop();
    EnterLineFollowing(20);
    //EnterLineFollowing(10);
    


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
    robotState = ROBOT_LINING;
}

void Robot::LineFollowingUpdate(void)
{
    if(robotState == ROBOT_LINING) 
    {
        // TODO: calculate the error in CalcError(), calc the effort, and update the motion
        float lineError = lineSensor.CalcError();

        float turnEffort = 1 * ((lineError * 0.5) + (0.1 * (lineError - prevLineError)));
        prevLineError = lineError;


        chassis.SetTwist(baseSpeed, turnEffort);        
    }
    if(robotState == ROBOT_CORRECTING){
        HandleCorrecting();
    }
}

/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */
void Robot::HandleIntersection(void)
{

    if(robotState == ROBOT_LINING) 
    {
        Serial.println("detected");

        //update location
        if(currDirection == 0){
            iGrid++;
        }
        else if (currDirection == 1){
            jGrid++;
        } else if (currDirection == 2){
            iGrid--;
        } 
        else if (currDirection == 3){
            jGrid--;
        }



        
        if (jGrid == jTarget && !jReached ){
            jReached = true;
            EnterCorrecting(1000);
            //EnterTurn(3);
            Serial.print(jGrid);
            Serial.print(jReached);
        } else if (jGrid != jTarget && !jReached){// reaches end, turns
            EnterLineFollowing(20);
        }  else if (jGrid == jTarget && jReached){
            
            if (iGrid == iTarget && !iReached ){
                iReached = true;
                EnterCorrecting(1000);
                Serial.print(iGrid);
                Serial.print(iReached);
            } else if (iGrid != iTarget && !iReached){// reaches end, turns
                EnterLineFollowing(20);
            }  else if (iGrid == iTarget && iReached){
                EnterTurn(2);
            }

        }

        


    }else{
        if(iGrid < iTarget){
            EnterTurn(0);
        }else if (iGrid == 0 && jGrid == 0 && iReached && jReached){
            EnterIdleState();
        } else if (iGrid == iTarget && iReached){
            jTarget = 0;
            jReached = false;
            iReached = false;
            EnterTurn(3);
        } else if(jGrid == jTarget){
            iTarget = 0;
            iReached = false;
            EnterTurn(2);
        }
    }
}

void Robot::EnterCorrecting(int time){
    robotState = ROBOT_CORRECTING;
    targetTime = millis() + time;
    HandleCorrecting();
}
void Robot::HandleCorrecting(void){
    
    if(currentTime < targetTime){
        chassis.SetWheelSpeeds(10,10);
        currentTime = millis();
    } else {
        chassis.SetWheelSpeeds(0,0);
        HandleIntersection();
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
    
    //Serial.println(lineSensor.CheckIntersection());

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

