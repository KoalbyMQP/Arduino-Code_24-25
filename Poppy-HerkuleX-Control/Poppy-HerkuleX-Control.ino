#include <Herkulex.h>
#include "Constants.h"
#include "ArduinoPoppy.h"


ArduinoPoppy robot;

void setup()
{
  robot.Setup();
}

void loop() {

  readCommand = robot.ReadCommand();

  switch (readCommand) {
    case Init:
      robot.Initialize(idArr);
      break;

    case GetPosition:
      robot.GetPosition();
      break;

    case SetPosition:
      robot.SetPosition(idArr);
      break;

    case SetPositionT:
      robot.SetPositionT(idArr);
      break;
    case ArmMirror:
      robot.ArmMirror(idArr, mirrorArray, armMirrorModeOn, lastMirror);
      break;

    default:
      break;

  }

  delay(20);

}
