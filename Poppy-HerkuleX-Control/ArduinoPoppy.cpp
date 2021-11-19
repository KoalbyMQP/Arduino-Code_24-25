#include "ArduinoPoppy.h"

ArduinoPoppy::ArduinoPoppy() {

}


void ArduinoPoppy::Setup() {
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");

  //Start Dynamixel shield
  // Set Port baudrate to 115200. This has to match with DYNAMIXEL baudrate.
  //dxl.begin(115200); //UNCOMMENT LINE WHEN USING SERIAL ADAPTOR - THIS CONFLICTS WITH USB SERIAL
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version - 1.0 for our motors
  dxl.setPortProtocolVersion(1.0);

  //Start HerkuleX
  Herkulex.beginSerial1(115200); //open serial port 1
  delay(100);
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
  for (int i = 0; i < MOTOR_COUNT; i++)
    if(idArr[i][TYPE_FIELD] == HERK)
      Herkulex.reboot(idArr[i][ID_FIELD]);
    else//Dynamixel
    {
      // Turn off torque when configuring items in EEPROM area
      dxl.torqueOff(idArr[i][ID_FIELD]);
      dxl.setOperatingMode(idArr[i][ID_FIELD], OP_POSITION);
      dxl.torqueOn(idArr[i][ID_FIELD]);
    }
  delay(500);
  //Move each motor to initialized position
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if(idArr[i][TYPE_FIELD] == HERK){
      Herkulex.torqueON(idArr[i][ID_FIELD]);
      Herkulex.moveOneAngle(idArr[i][ID_FIELD], idArr[i][3], 1000, LED_BLUE);
      //I cannot explain why this line is needed, but I swear on my life removing it makes the motor stop working right in init
      Serial.println(Herkulex.getAngle(idArr[i][ID_FIELD]));
    }else{
      //Not tested
      dxl.setGoalPosition(idArr[i][ID_FIELD], idArr[i][3]);
    }
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
  int mappedTarget = map(positionPerc, 0, 100, idArr[motorNum][1], idArr[motorNum][2]);//map 0-100 input to the motor's range
  if(idArr[motorNum][TYPE_FIELD] == HERK)
    Herkulex.moveOneAngle(idArr[motorNum][0], mappedTarget, 1000, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    dxl.setGoalPosition(idArr[motorNum][0], mappedTarget);
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
  int mappedTarget = map(positionPerc, 0, 100, idArr[motorNum][1], idArr[motorNum][2]);//map 0-100 input to the motor's range
  if(idArr[motorNum][TYPE_FIELD] == HERK)
    Herkulex.moveOneAngle(idArr[motorNum][0], mappedTarget, tTime, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    //TODO: timed motrion for Dynamixel
    dxl.setGoalPosition(idArr[motorNum][0], mappedTarget);
}

//Caution: might move FAST when starting, no safties currently implemented for that
void ArduinoPoppy::ArmMirror(/*int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror*/) { //Set position, use default time of motion
  //Assumes all arm motors being used in this function are HerkuleX motors, not Dynamixels
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

      Herkulex.moveOneAngle(idArr[rowSet][0], map(pos1, idArr[rowRead][1], idArr[rowRead][2], idArr[rowSet][1], idArr[rowSet][2]), 100, LED_BLUE); //move motor
    }

    //delay(100);
  }
}
