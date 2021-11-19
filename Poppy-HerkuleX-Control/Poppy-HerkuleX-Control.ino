#include <Herkulex.h>
#include "ArduinoPoppy.h"

ArduinoPoppy robot;

void setup()
{
  robot.Setup();
}

void loop() {

  robot.command = robot.ReadCommand();

  switch (robot.command) {
    case Init:
      robot.Initialize();
      break;

    case GetPosition:
      robot.GetPosition();
      break;

    case SetPosition:
      robot.SetPosition();
      break;

    case SetPositionT:
      robot.SetPositionT();
      break;
      
    case ArmMirror:
      robot.ArmMirror();
      break;
      
    case Shutdown:
      robot.Shutdown();
      break;
      
    default:
      break;
  }

  //Do other tasks
  robot.UpdateRobot();
  delay(20);

}
