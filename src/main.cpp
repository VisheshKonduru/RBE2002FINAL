#include <Arduino.h>
#include <LSM6.h>
#include "robot.h"
#include "Pathfinding.h"

//Pathfinding pathfinder;

Robot robot;

void setup() 
{
  Serial.begin(250000);
  Serial.println("setup");

#ifdef __LOOP_DEBUG__
  while(!Serial){delay(5);}
#endif

  robot.InitializeRobot();

  // int i_start = 1, j_start = 0;
  // int i_end = 2, j_end = 4;

  // pathfinder.findPath(i_start, j_start, i_end, j_end);

}

void loop() 
{

  robot.RobotLoop();
}
