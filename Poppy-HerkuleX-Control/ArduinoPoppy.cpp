#include "ArduinoPoppy.h"

ArduinoPoppy::ArduinoPoppy() {

}


void ArduinoPoppy::Setup() {
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1
  delay(500);
  //Herkulex.initialize(); //initialize motors
  delay(200);
  pinMode(A0, INPUT);
}

int ArduinoPoppy::ReadCommand() {
  int readCommand = -1;
  if (Serial.available() != 0) {
    readCommand = Serial.parseInt();
    Serial.print("Command: ");
    Serial.println(readCommand);
  }
  return readCommand;
}

void ArduinoPoppy::Initialize() {
  Herkulex.initialize(); //initialize motors
  for (int i = 0; i < sizeof(idArr); i++)
    Herkulex.reboot(idArr[i][0]);
  delay(500);
  //Move each motor to initialized position
  for (int i = 0; i < sizeof(idArr); i++) {
    Herkulex.torqueON(idArr[i][0]);
    Herkulex.moveOneAngle(idArr[i][0], idArr[i][3], 1000, LED_BLUE);
    //I cannot explain why this line is needed, but I swear on my life removing it makes the motor stop working right in init
    Serial.println(Herkulex.getAngle(idArr[i][0]));

  }
}

void ArduinoPoppy::GetPosition() { // Todo
  Serial.println("Motor position here");
}

void ArduinoPoppy::SetPosition() { //Set position, use default time of motion
  //Read motor number
  Serial.println("Enter Motor Index ");        //Prompt User for input
  while (Serial.available() == 0) {}          // wait for user input
  int motorNum = Serial.parseInt();                    //Read user input and hold it in a variable

  //Read motor target position
  Serial.println("Enter Motor Position ");
  while (Serial.available() == 0) {}
  int positionPerc = Serial.parseInt();

  //Send parsed command to the motor
  Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc, 0, 100, idArr[motorNum][1], idArr[motorNum][2]), 1000, LED_BLUE); //move motor with 300 speed
}

void ArduinoPoppy::SetPositionT() { //Set position with time of motion
  //Read motor number
  Serial.println("Enter Motor Index ");
  while (Serial.available() == 0) {}
  int motorNum = Serial.parseInt();

  //Read motor target position
  Serial.println("Enter Motor Position ");
  while (Serial.available() == 0) {}
  int positionPerc = Serial.parseInt();

  //Read time of motion
  Serial.println("Enter travel time (millis) ");
  while (Serial.available() == 0) {}
  int tTime = Serial.parseInt();

  //Send parsed command to the motor
  Herkulex.moveOneAngle(idArr[motorNum][0], map(positionPerc, 0, 100, idArr[motorNum][1], idArr[motorNum][2]), tTime, LED_BLUE); //move motor with 300 speed
}

void ArduinoPoppy::ArmMirror(/*int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror*/) { //Set position, use default time of motion
  //Read motor number
  Serial.println("Enter Value (0 off, 1 on) ");        //Prompt User for input
  while (Serial.available() == 0) {}          // wait for user input
  int motorNum = Serial.parseInt();

  if (motorNum) {
    armMirrorModeOn = true;
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueOFF(idArr[mirrorArray[i][0]][0]);
    }
  } else {
    armMirrorModeOn = false;
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueON(idArr[mirrorArray[i][0]][0]);
    }
  }
}

void ArduinoPoppy::UpdateRobot() {
  if (armMirrorModeOn) {
    lastMirror = millis();
    for (int i = 0; i < 4; i++) {
      int rowRead = mirrorArray[i][0];
      int rowSet = mirrorArray[i][1];
      int pos1 = Herkulex.getAngle(idArr[rowRead][0]);

      //Serial.println(map(pos1,idArr[rowRead][1],idArr[rowRead][2],idArr[rowSet][1],idArr[rowSet][2]));

      Herkulex.moveOneAngle(idArr[rowSet][0], map(pos1, idArr[rowRead][1], idArr[rowRead][2], idArr[rowSet][1], idArr[rowSet][2]), 500, LED_BLUE); //move motor
    }

    //delay(100);
  }
}
