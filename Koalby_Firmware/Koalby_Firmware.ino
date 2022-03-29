//Main firmware for Koalby humanoid robot

#include <Herkulex.h>
#include "ArduinoPoppy.h"

//Defined in ArduinoPoppy.h, motor control methods
ArduinoPoppy robot;

//Set up 
void setup()
{
  robot.Setup();
}

void loop() {

  robot.command = robot.ReadCommand();
  #ifdef DEBUG 
  if(robot.command != -1)
  Serial.println(robot.command);
  #endif

  switch (robot.command) {
    case Init:
      robot.Initialize();
      #ifdef DEBUG 
      Serial.println("\nINIT");
      #endif
      break;

    case GetPosition:
      robot.GetPosition();
      #ifdef DEBUG 
      Serial.println("GET POSITION");
      #endif
      break;

    case SetPosition:
      robot.SetPosition();
      #ifdef DEBUG 
      Serial.println("SET POSITION");
      #endif
      break;

    case SetPositionT:
      robot.SetPositionT();
      break;
      
    case ArmMirror:
      robot.ArmMirror();
      break;

    case SetTorque:
      robot.SetTorque();
      break;

    case SetCompliant:
      robot.SetCompliant();
      break;
      
    case Shutdown:
      robot.Shutdown();
      break;
      
    default:
      break;
  }



  //Do other tasks
  robot.UpdateRobot();
  delay(5);

}
