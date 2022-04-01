#include "ArduinoPoppy.h"

ArduinoPoppy::ArduinoPoppy() {

}


void ArduinoPoppy::Setup() {
  delay(2000);  //a delay to have time for serial monitor opening
  SERIAL_MONITOR.begin(115200);    // Open serial communications
  #ifdef HUMAN_CONTROL
  SERIAL_MONITOR.println("Begin");
  #endif

  //Start Dynamixel shield
  // Set Port baudrate to 115200. This has to match with DYNAMIXEL baudrate.
  #ifdef DYNAMIXEL_CONTROL
  dxl.begin(115200); 
  #endif
  
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version - 1.0 for our motors
  dxl.setPortProtocolVersion(1.0); //This differs for Dynamixel 320's, will need to check before each command

  //Start HerkuleX
  Herkulex.beginSerial1(115200); //open serial port 1 for HerkuleX's
  delay(100);
}

int ArduinoPoppy::ReadCommand() {
  int readCommand = -1;
  if (SERIAL_MONITOR.available() != 0) {
    readCommand = SERIAL_MONITOR.parseInt();
    #ifdef HUMAN_CONTROL
    SERIAL_MONITOR.print("Command: ");
    SERIAL_MONITOR.println(readCommand);
    #endif
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
      Herkulex.getAngle(idArr[i].hexID);
    } else {
      dxl.setGoalPosition(idArr[i].hexID, idArr[i].homePos,UNIT_DEGREE);
    }
  }
}

//TODO - update for DXL
void ArduinoPoppy::Shutdown() {
  #ifdef HUMAN_CONTROL
  SERIAL_MONITOR.println("Robot Shutdown");
  #endif
  
  for (int motorNum = 0; motorNum < MOTOR_COUNT; motorNum++) {
    if (idArr[motorNum].type == HERK) {
      Herkulex.torqueOFF(idArr[motorNum].hexID);
      Herkulex.setLed(idArr[motorNum].hexID, LED_RED); 
    }else{
      dxl.torqueOff(idArr[motorNum].hexID);
    }
  }
}

// Return position in the same range as setPosition
void ArduinoPoppy::GetPosition() {
  //Serial.println("get position");
  // Get motor id
  int motorNum =  getIntFromSerial("Enter Motor Id");

  //Print the Angle
  //UPDATE: returns distance from home
  if (idArr[motorNum].type == HERK) {
    
    float angle=0;
    int attempt=0;
    do{
      angle = Herkulex.getAngle(idArr[motorNum].hexID);
      attempt++;
    }while(angle<-164 && attempt<10);
    //Serial2.println(angle);
    //Serial2.println(map(angle,    idArr[motorNum].minPos,idArr[motorNum].maxPos,  0,100));
    if(idArr[motorNum].minPos<idArr[motorNum].maxPos)
      SERIAL_MONITOR.println((int)(angle-idArr[motorNum].homePos));
    else
      SERIAL_MONITOR.println((int)(-angle+idArr[motorNum].homePos));

  } else {
    float angle=0;
    angle = dxl.getPresentPosition(idArr[motorNum].hexID, UNIT_DEGREE);
    
    //Serial2.println(angle);
    //Serial2.println(map(angle,    idArr[motorNum].minPos,idArr[motorNum].maxPos,  0,100));
    if(idArr[motorNum].minPos<idArr[motorNum].maxPos)
      SERIAL_MONITOR.println((int)(angle-idArr[motorNum].homePos));
    else
      SERIAL_MONITOR.println((int)(-angle+idArr[motorNum].homePos));
  }
}

void ArduinoPoppy::SetPosition() { //Set position, use default time of motion
  //Read motor number
  int motorNum = getIntFromSerial("Enter Motor Index ");   
  /*Serial.print("out: ");
  Serial.println(motorNum);*/

  //Read motor target position
  int positionPerc = getIntFromSerial("Enter Motor Position ");

  //Send parsed command to the motor
  int mappedTarget=0;
  //Account for motor direction when setting limits
  if(positionPerc,idArr[motorNum].minPos<idArr[motorNum].maxPos){
    positionPerc = positionPerc+idArr[motorNum].homePos;
    mappedTarget = min(max(positionPerc,idArr[motorNum].minPos),idArr[motorNum].maxPos); 
  }else{
    positionPerc = -positionPerc+idArr[motorNum].homePos;
    mappedTarget = max(min(positionPerc,idArr[motorNum].minPos),idArr[motorNum].maxPos); 
    /*SERIAL_MONITOR.print("Val2: ");
    SERIAL_MONITOR.print(positionPerc);
    SERIAL_MONITOR.print("actual: ");
    SERIAL_MONITOR.println(mappedTarget);*/
  }
      
  if (idArr[motorNum].type == HERK)
    Herkulex.moveOneAngle(idArr[motorNum].hexID, mappedTarget, 1000, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    dxl.setGoalPosition(idArr[motorNum].hexID, mappedTarget, UNIT_DEGREE);
}

void ArduinoPoppy::SetPositionT() { //Set position with time of motion
  //Read motor number}
  int motorNum = getIntFromSerial("Enter Motor Index ");

  //Read motor target position
  int positionPerc = getIntFromSerial("Enter Motor Position ");

  //Read time of motion
  int tTime = getIntFromSerial("Enter travel time (millis) ");

  //Send parsed command to the motor
  int mappedTarget=0;
  //Account for motor direction when setting limits
  if(positionPerc,idArr[motorNum].minPos<idArr[motorNum].maxPos){
    positionPerc = positionPerc+idArr[motorNum].homePos;
    mappedTarget = min(max(positionPerc,idArr[motorNum].minPos),idArr[motorNum].maxPos); 
    /*SERIAL_MONITOR.print("Val: ");
    SERIAL_MONITOR.print(positionPerc);
    SERIAL_MONITOR.print("actual: ");
    SERIAL_MONITOR.println(mappedTarget);*/
  }else{
    positionPerc = -positionPerc+idArr[motorNum].homePos;
    mappedTarget = max(min(positionPerc,idArr[motorNum].minPos),idArr[motorNum].maxPos); 
    /*SERIAL_MONITOR.print("Val2: ");
    SERIAL_MONITOR.print(positionPerc);
    SERIAL_MONITOR.print("actual: ");
    SERIAL_MONITOR.println(mappedTarget);*/
  }
      
  if (idArr[motorNum].type == HERK)
    Herkulex.moveOneAngle(idArr[motorNum].hexID, mappedTarget, tTime, LED_BLUE); //move motor with 300 speed
  else //Dynamixel
    //TODO: timed motion for Dynamixel
    dxl.setGoalPosition(idArr[motorNum].hexID, mappedTarget, UNIT_DEGREE);
}

//Caution: might move FAST when starting, no safties currently implemented for that
void ArduinoPoppy::ArmMirror(/*int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror*/) { //Set position, use default time of motion
  //Assumes all arm motors being used in this function are HerkuleX motors, not Dynamixels
  //Read setting number
  armMirrorModeOn = getIntFromSerial("Enter Value (0 off, 1 on) ");

  if (armMirrorModeOn) {
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueOFF(idArr[mirrorArray[i][0]].hexID);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      Herkulex.torqueON(idArr[mirrorArray[i][0]].hexID);
    }
  }
}

void ArduinoPoppy::SetTorque() { //Set position, use default time of motion
  //Read motor number
  #ifdef HUMAN_CONTROL
  SERIAL_MONITOR.println("Enter Motor Index ");        //Prompt User for input
  #endif
  
  while (SERIAL_MONITOR.available() == 0) {}          // wait for user input
  int motorNum = SERIAL_MONITOR.parseInt();                    //Read user input and hold it in a variable

  //Read motor target position
  int setTorqueOn = getIntFromSerial("Enter Motor Torque(0 = off, 1 = on) ");

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

void ArduinoPoppy::SetCompliant() { //Set position, use default time of motion
  //Read motor number
  #ifdef HUMAN_CONTROL
  SERIAL_MONITOR.println("Enter Motor Index ");        //Prompt User for input
  #endif
  
  while (SERIAL_MONITOR.available() == 0) {}          // wait for user input
  int motorNum = SERIAL_MONITOR.parseInt();                    //Read user input and hold it in a variable

  //Read value
  int setTorqueOn = getIntFromSerial("Enter Motor Compliance (0 = off, 1 = on) ");

  //Send parsed command to the motor
  if (idArr[motorNum].type == HERK)
    if(setTorqueOn){
      Serial.print("Set to compliant: ");
      Serial.println(motorNum);
      compliantMotorSet.add(motorNum);
    }else
      compliantMotorSet.sub(motorNum);
  else //Dynamixel
  {
    //TODO implement
  }
}

void ArduinoPoppy::UpdateRobot() {
  if (armMirrorModeOn) {
    lastMirror = millis();
    for (int i = 0; i < 4; i++) {
      int rowRead = mirrorArray[i][0];
      int rowSet = mirrorArray[i][1];
      int pos1 = Herkulex.getAngle(idArr[rowRead].hexID)-idArr[rowRead].homePos+idArr[rowSet].homePos;

      if(pos1>-164)
      {
        
          //Send parsed command to the motor
          int mappedTarget = min(max(pos1,idArr[rowSet].minPos),idArr[rowSet].maxPos);
          Herkulex.moveOneAngle(idArr[rowSet].hexID, mappedTarget, 200, LED_BLUE); //move motor
      }
    }
  }

  //Iterate over compliant motors
  int n = compliantMotorSet.first();
  
  while (n != -1)
  {
    if(compliancePWMCounter%10 < 2)
      Herkulex.torqueOFF(idArr[n].hexID);
    else
      Herkulex.torqueON(idArr[n].hexID);
    Herkulex.moveOneAngle(idArr[n].hexID, Herkulex.getAngle(idArr[n].hexID), 200, 2);
    n = compliantMotorSet.next();
  }
  compliancePWMCounter++;
}

//Return an integer entered over serial - options with and without a message
int ArduinoPoppy::getIntFromSerial(){
  while (SERIAL_MONITOR.available() == 0) {}
  return SERIAL_MONITOR.parseInt();
}

int ArduinoPoppy::getIntFromSerial(char* msg){
  #ifdef HUMAN_CONTROL
  SERIAL_MONITOR.println(msg);
  #endif
  
  return getIntFromSerial();
}
