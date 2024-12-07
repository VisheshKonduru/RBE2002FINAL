// #include "robot.h"









// // #ifndef PATH_PLANNING_H
// // #define PATH_PLANNING_H

// // using namespace std;

// // const int GRID_ROWS = 3;
// // const int GRID_COLS = 6;
// // int intersections[GRID_ROWS][GRID_COLS] = {
// //     {0, 0, 0, 0, 0, 0},
// //     {0, 0, 0, 0, 0, 0},
// //     {0, 0, 0, 0, 0, 0},
// // };

// // int jPaths[GRID_ROWS][GRID_COLS - 1] = {
// //     {0, 0, 0, 0, 0},
// //     {0, 0, 0, 0, 0},
// //     {0, 0, 0, 0, 0},
// // };

// // int iPaths[GRID_ROWS - 1][GRID_COLS] = {
// //     {0, 0, 0, 0, 0, 0},
// //     {0, 0, 0, 0, 0, 0},
// // };

// // int fullPaths[GRID_ROWS + (GRID_ROWS - 1)][GRID_COLS + (GRID_COLS - 1)] = {
// //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
// //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
// //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
// //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
// //     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
// // };



// // /**
// //  * Functions related to the IMU (turning; ramp detection)
// //  */
// // // void Robot::EnterTurn(int turns)
// // // {
// // //     Serial.print("--> TURN (");
// // //     Serial.print(turns);
// // //     Serial.println(')');
// // //     robotState = ROBOT_TURNING;

// // //     startAngle = eulerAngles.z;
// // //     turnAngle = 90 * turns;
// // //     if(turns > 0)
// // //     {
// // //         chassis.SetTwist(0, 0.5);
// // //     }

// // //     else if(turns < 0)
// // //     {
// // //         chassis.SetTwist(0, -0.5);
// // //     }
// // // }

// // void Robot::EnterTurn(int desiredDirection)
// // {
// //     Serial.println(" -> TURN");
// //     robotState = ROBOT_TURNING;

// //     /**
// //      * TODO: Add code to initiate the turn and set the target
// //      */
// //     targetDirection = desiredDirection;
// //     turns =  (targetDirection - currDirection);

    
// //     if (turns >  0){
// //         chassis.SetTwist(0,1);
// //         targetHeading = eulerAngles.z + (abs(turns) * 90);
// //     } else {
// //         chassis.SetTwist(0,-1);
// //         targetHeading = eulerAngles.z - (abs(turns) * 90);
// //     }
    
// //     Serial.println(targetHeading);
// // }

// // // // Now a proper checker...
// // // bool Robot::CheckTurnComplete(void)
// // // {
// // //     bool retVal = false;
    
// // //     static bool prevPast = false;
// // //     bool past = false;

// // //     if(turnAngle > 0)
// // //     {
// // //         past = (eulerAngles.z - startAngle > turnAngle);
// // //         if(past && !prevPast) retVal = true;
// // //     }

// // //     else
// // //     {
// // //         past = (eulerAngles.z - startAngle < turnAngle);
// // //         if(past && !prevPast) retVal = true;
// // //     }

// // //     prevPast = past;

// // //     return retVal;
// // // }
// // bool Robot::CheckTurnComplete(void)
// // {

// //     /**
// //      * TODO: add a checker to detect when the turn is complete
// //      */
// //     if (targetDirection - currDirection > 0){
// //         if (eulerAngles.z > targetHeading){
// //         return true;
// //         Serial.println("checking");
// //         } 

// //     } else if (targetDirection - currDirection < 0){
// //         if(eulerAngles.z < targetHeading) {
// //             return true;
// //             Serial.println("checking");
// //         }

// //     } else {
// //         return false;
// //     }

// // }



// // void Robot::HandleTurnComplete(void)
// // {
// //     if(robotState == ROBOT_TURNING)
// //     {
// //         Serial.println("turn completed");
// //         currDirection = targetDirection;
// //         turns = 0;
// //         Serial.println(currDirection);
// //         chassis.Stop();
// //         EnterLineFollowing(baseSpeed);
// //     }
// // }

// // /**
// //  * Functions related to line following and intersection detection.
// //  */
// // void Robot::EnterLineFollowing(float speed) 
// // {
// //     Serial.println(" -> LINING"); 
// //     baseSpeed = speed; 
// //     robotState = ROBOT_LINING;
// // }

// // void Robot::LineFollowingUpdate(void)
// // {
// //     if(robotState == ROBOT_LINING)//|| robotState == ROBOT_CENTERING) 
// //     {
// //         // Read line sensors
// //         int16_t lineError = lineSensor.CalcError();
// //         float turnEffort = lineError * 0.001;

// //         chassis.SetTwist(baseSpeed, -turnEffort);
// //     }
// // }

// // void Robot::HandleIntersection(void)
// // {
// //     Serial.print("X -- ");
// //     if(robotState == ROBOT_LINING) 
// //     {
// //         if(currDirection == 0){
// //             iGrid++;
// //         }
// //         else if (currDirection == 1){
// //             jGrid++;
// //         } else if (currDirection == 2){
// //             iGrid--;
// //         } 
// //         else if (currDirection == 3){
// //             jGrid--;
// //         }
        
// //         Serial.print("Now at: ");
// //         Serial.print(iGrid);
// //         Serial.print(',');
// //         Serial.print(jGrid);
// //         Serial.print('\n');

// //         /* Before we turn, we'll center the robot on the intersection. Creep at 1.5cm/s for 3 secs. */
// //         chassis.SetTwist(1.5, 0);
// //         centeringTimer.start(3000);
// //         robotState = ROBOT_CENTERING;
// //         Serial.println("--> CENTER");
// //     }
// // }

// // bool Robot::CheckCenteringComplete(void)
// // {
// //     return centeringTimer.checkExpired();
// // }

// // void Robot::HandleCenteringComplete(void)
// // {
// //     if(robotState == ROBOT_CENTERING)
// //     {
// //         /**
// //          * Now that we're centered, we can work through the logic of where to go.
// //          * 
// //          * I'll drive to the correct j first, then i.
// //          */
// //         if(jGrid == jTarget)
// //         {
// //             if(iGrid == iTarget) // reached destination!
// //             {
// //                 Serial.println("Reached Dest!");
// //                 EnterIdleState();
// //                 return;
// //             }
// //             else if(iGrid < iTarget) // we'll need to turn EAST
// //             {
// //                 targetDirection = 0;
// //             }
// //             else // need to go WEST
// //             {
// //                 targetDirection = 2;
// //             }
// //         }
// //         else if(jGrid < jTarget)
// //         {
// //             targetDirection = 1;
// //         }
// //         else
// //         {
// //             targetDirection = 3;
// //         }

// //         if(currDirection == targetDirection) // we're headed in the right direction
// //         {   
// //             EnterLineFollowing(baseSpeed);
// //         }

// //         else
// //         {
// //             int8_t turnCount = targetDirection - currDirection;

// //             // take the shortest path
// //             if(turnCount > 2) turnCount -= 4;
// //             if(turnCount < -2) turnCount += 4;

// //             EnterTurn(turnCount);
// //         }
// //     }
// // }