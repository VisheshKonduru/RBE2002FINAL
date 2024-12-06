#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>
#include <Romi32U4Buttons.h>
#include <OpenMV.h>
#include <apriltagdatum.h>
#include <event_timer.h>

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
        CTRL_SETUP,
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_TELEOP;

    EventTimer centeringTimer;
    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE, 
        ROBOT_LINING,
        ROBOT_TURNING,
        ROBOT_CORRECTING,
        ROBOT_CENTERING,
        ROBOT_SEARCHING,    // these are for camera code
        ROBOT_APPROACHING, // these are both for camera code

    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    /* Line sensor */
    LineSensor lineSensor;

    /* To add later: rangefinder, camera, etc.*/
    int16_t prevLineError = 0;


    // For managing key presses
    String keyString;

    /**
     * The LSM6 IMU that is included on the Romi. We keep track of the orientation through
     * Euler angles (roll, pitch, yaw).
     */
    LSM6 imu;
    LSM6::vector<float> prevEulerAngles;
    LSM6::vector<float> eulerAngles;

    /* targetHeading is used for commanding the robot to turn */
    float targetHeading;

    /* baseSpeed is used to drive at a given speed while, say, line following.*/
    float baseSpeed = 0;

    // /**
    //  * For tracking the motion of the Romi. We keep track of the intersection we came
    //  * from and the one we're headed to. You'll program in the map in handleIntersection()
    //  * and other functions.
    //  */
    // enum INTERSECTION {NODE_START, NODE_1, NODE_2, NODE_3,};
    // INTERSECTION nodeFrom = NODE_START;
    // INTERSECTION nodeTo = NODE_1;

    uint8_t iGrid = 1;
    uint8_t jGrid = 0;
    int8_t startAngle = 90;
    int8_t turnAngle = 0;
    int8_t currDirection = 1;
    int8_t targetDirection = 1;
    int turns = 0;
    
    
    // EAST -> 0; NORTH -> 1; WEST = 2; SOUTH -> 3
    //int8_t targetDirection = 1;

    enum DIRECTION {
        EAST,
        NORTH,
        WEST,
        SOUTH,
    };

    // DIRECTION currDirection = NORTH;
    

    //direction %= 4;

    uint8_t iTarget = 0;
    uint8_t jTarget = 0;
    int8_t numTurns = 0;
    bool iReached = false;
    bool jReached = false;

    float currentTime = 0;
    float targetTime = 0;   

    // // camera shit 
    int detectedTagID; // var to store tag ID
    AprilTagDatum currentTag; //stores latest detected tag
    bool tagDetected = false; //flag to indicate if a tag has been stored 

    //PID for approaching state 
    float approachKp = 0.014;
    float maxApproachSpeed = 0.5;

    // new members for non-blocking
    unsigned long previousMillis;
    const long interval = 500;
    int blinkCount;
    int tagID;
    
public:
    Robot(void) {keyString.reserve(8);} //reserve some memory to avoid reallocation
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);
    void Setup(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);
    void EnterSetupMode(void);

    /**
     * Line following and navigation routines.
     */
    void EnterLineFollowing(float speed);
    void LineFollowingUpdate(void);

    bool CheckIntersection(void) {return lineSensor.CheckIntersection();}
    void HandleIntersection(void);
    void EnterCorrecting(int time);
    void HandleCorrecting(void);

    void EnterTurn(int desiredTurns);
    bool CheckTurnComplete(void);
    void HandleTurnComplete(void);
    bool CheckCenteringComplete(void);
    void HandleCenteringComplete(void);



    // /*All camera shit */
    void HandleAprilTag(const AprilTagDatum& tag);
    void ApproachTag (void);
    void EnterSearchingState(void);
    void EnterApproachingState(void);
    bool CheckApproachComplete(int headingTolerance, int distanceTolerance);

    OpenMV openMV;
    EventTimer tagLossTimer;
    const unsigned long tagLossTimeout = 1000; 

    /* IMU routines */
    void HandleOrientationUpdate(void);

    /* For commanding the lifter servo */
    void SetLifter(uint16_t position);

    
};
