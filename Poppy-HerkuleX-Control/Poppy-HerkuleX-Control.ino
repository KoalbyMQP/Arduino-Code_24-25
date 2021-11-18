#include <Herkulex.h>
#include "ArduinoPoppy.h"

int readCommand = 0;

ArduinoPoppy robot;

void setup()
{
  robot.Setup();
}

void loop() {

  readCommand = robot.ReadCommand();

  switch (readCommand) {
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
    default:
      break;
  }

  //Do other tasks
  robot.UpdateRobot();
  delay(20);

}
