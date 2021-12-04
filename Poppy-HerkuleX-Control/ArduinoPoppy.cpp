#include "ArduinoPoppy.h"

ArduinoPoppy::ArduinoPoppy() {

}


void ArduinoPoppy::Setup() {
  delay(2000);  //a delay to have time for serial monitor opening
  Serial2.begin(115200);    // Open serial communications
  Serial2.println("Begin");

  //Start Dynamixel shield
  // Set Port baudrate to 115200. This has to match with DYNAMIXEL baudrate.
  dxl.begin(115200); //UNCOMMENT LINE WHEN USING SERIAL ADAPTOR - THIS CONFLICTS WITH USB SERIAL
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version - 1.0 for our motors
  dxl.setPortProtocolVersion(1.0);

  //Start HerkuleX
  Herkulex.beginSerial1(115200); //open serial port 1
  delay(100);
}

int ArduinoPoppy::ReadCommand() {
  int readCommand = -1;
  if (Serial2.available() != 0) {
    readCommand = Serial2.parseInt();
    Serial2.print("Command: ");
    Serial2.println(readCommand);
  }
  return readCommand;
}

void ArduinoPoppy::Initialize() {
  Herkulex.initialize(); //initialize motors
  for (int i = 0; i < MOTOR_COUNT; i++)
    if (idArr[i].type == HERK)
      Herkulex.reboot(idArr[i].hexID);
    else//Dynamixel
    {
      // Turn off torque when configuring items in EEPROM area
      dxl.torqueOff(idArr[i].hexID);
      dxl.setOperatingMode(idArr[i].hexID, OP_POSITION);
      dxl.torqueOn(idArr[i].hexID);
    }
  delay(500);
  //Move each motor to initialized position
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (idArr[i].type == HERK) {
      Herkulex.torqueON(idArr[i].hexID);
      Herkulex.moveOneAngle(idArr[i].hexID, idArr[i].homePos, 1000, LED_BLUE);
      //I cannot explain why this line is needed, but I swear on my life removing it makes the motor stop working right in init
      Serial.println(Herkulex.getAngle(idArr[i].hexID));
    } else {
      //Not tested
      dxl.setGoalPosition(idArr[i].hexID, idArr[i].homePos,UNIT_DEGREE);
    }
  }
}

void ArduinoPoppy::Shutdown() {
  Serial2.println("Robot Shutdown");
  for (int motorNum = 0; motorNum < MOTOR_COUNT; motorNum++) {
    Herkulex.torqueOFF(idArr[motorNum].hexID);
    Herkulex.setLed(idArr[motorNum].hexID, LED_RED); 
  }
}

//In progress
void ArduinoPoppy::GetPosition() {
  // Get motor id
  Serial2.println("Enter Motor Id");
  while (Serial.available() == 0) {}
  int motorNum =  Serial.parseInt();


  //TODO - return in 0-100 range and support Dynamixel motors
  //Print the Angle - This should return in the same range (0-100) as set position for the Pi
    Serial2.println(Herkulex.getAngle(idArr[motorNum].hexID));
}

void ArduinoPoppy::SetPosition() { //Set position, use default time of motion
  //Read motor number
  Serial2.println("Enter Motor Index ");        //Prompt User for input
  while (Serial2.available() == 0) {}          // wait for user input
  int motorNum = Serial2.parseInt();                    //Read user input and hold it in a variable

  //Read motor target position
  Serial2.println("Enter Motor Position ");
  while (Serial2.available() == 0) {}
  int positionPerc = Serial2.parseInt();

  //Send parsed command to the motor
  int mappedTarget = map(positionPerc, 0, 100, idArr[motorNum].minPos, idArr[motorNum].maxPos);//map 0-100 input to the motor's range
  if (idArr[motorNum].type == HERK)
    Herkulex.moveOneAngle(idArr[motorNum].hexID, mappedTarget, 1000, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    dxl.setGoalPosition(idArr[motorNum].hexID, mappedTarget, UNIT_DEGREE);
}

void ArduinoPoppy::SetPositionT() { //Set position with time of motion
  //Read motor number
  Serial2.println("Enter Motor Index ");
  while (Serial2.available() == 0) {}
  int motorNum = Serial2.parseInt();

  //Read motor target position
  Serial2.println("Enter Motor Position ");
  while (Serial2.available() == 0) {}
  int positionPerc = Serial2.parseInt();

  //Read time of motion
  Serial2.println("Enter travel time (millis) ");
  while (Serial2.available() == 0) {}
  int tTime = Serial2.parseInt();

  //Send parsed command to the motor
  int mappedTarget = map(positionPerc, 0, 100, idArr[motorNum].minPos, idArr[motorNum].maxPos);//map 0-100 input to the motor's range
  if (idArr[motorNum].type == HERK)
    Herkulex.moveOneAngle(idArr[motorNum].hexID, mappedTarget, tTime, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    //TODO: timed motrion for Dynamixel
    dxl.setGoalPosition(idArr[motorNum].hexID, mappedTarget, UNIT_DEGREE);
}

//Caution: might move FAST when starting, no safties currently implemented for that
void ArduinoPoppy::ArmMirror(/*int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror*/) { //Set position, use default time of motion
  //Assumes all arm motors being used in this function are HerkuleX motors, not Dynamixels
  //Read motor number
  Serial2.println("Enter Value (0 off, 1 on) ");        //Prompt User for input
  while (Serial2.available() == 0) {}          // wait for user input
  int motorNum = Serial2.parseInt();

  if (motorNum) {
    armMirrorModeOn = true;
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueOFF(idArr[mirrorArray[i][0]].hexID);
    }
  } else {
    armMirrorModeOn = false;
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueON(idArr[mirrorArray[i][0]].hexID);
    }
  }
}

void ArduinoPoppy::SetTorque() { //Set position, use default time of motion
  //Read motor number
  Serial2.println("Enter Motor Index ");        //Prompt User for input
  while (Serial2.available() == 0) {}          // wait for user input
  int motorNum = Serial2.parseInt();                    //Read user input and hold it in a variable

  //Read motor target position
  Serial2.println("Enter Motor Torque(0 = off, 1 = on) ");
  while (Serial2.available() == 0) {}
  int setTorqueOn = Serial2.parseInt();

  //Send parsed command to the motor
  if (idArr[motorNum].type == HERK)
    if(setTorqueOn)
      Herkulex.torqueON(idArr[motorNum].hexID);
    else
      Herkulex.torqueOFF(idArr[motorNum].hexID);
  else //Dynamixel
    if(setTorqueOn)
      dxl.torqueOn(idArr[motorNum].hexID);
    else
      dxl.torqueOff(idArr[motorNum].hexID);
}

void ArduinoPoppy::UpdateRobot() {
  if (armMirrorModeOn) {
    lastMirror = millis();
    for (int i = 0; i < 4; i++) {
      int rowRead = mirrorArray[i][0];
      int rowSet = mirrorArray[i][1];
      int pos1 = Herkulex.getAngle(idArr[rowRead].hexID);

      Herkulex.moveOneAngle(idArr[rowSet].hexID, map(pos1, idArr[rowRead].minPos, idArr[rowRead].maxPos, idArr[rowSet].minPos, idArr[rowSet].maxPos), 200, LED_BLUE); //move motor
    }
  }
}
