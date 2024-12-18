#include <Herkulex.h>
#include <Ava.h>

int motionTime = 400;
int delayTime = 350;

//Checks which motors are attached and reading real values
void setup()  
{
  delay(100);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial3(115200); //open serial port 1  

  for(int i =0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
 
  Herkulex.torqueON(0xfe);
  delay(100);

  // non moving
  for (int i = 0; i < motorsLen; i++) {
    Herkulex.torqueON(motors[i].hexID);
    Herkulex.moveOneAngle(motors[i].hexID, motors[i].homePos, 1000, 0, motors[i].is0601);
  }

  // Move arms down and forward a little
  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 75, 1000, 0, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 75, 1000, 0, Right_Shoulder.is0601);
  Herkulex.moveOneAngle(Left_Arm_Abductor.hexID, Left_Arm_Abductor.homePos + 70, 1000, 0, Left_Arm_Abductor.is0601);
  Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, Right_Arm_Abductor.homePos - 70, 1000, 0, Right_Arm_Abductor.is0601);
  Herkulex.moveOneAngle(Left_Arm_Rotator.hexID, Left_Arm_Rotator.homePos + 180, 1000, 0, Left_Arm_Rotator.is0601);
  Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, Right_Arm_Rotator.homePos - 180, 1000, 0, Right_Arm_Rotator.is0601);

  delay(1100);
}

// angles and timing not tuned well :)
void loop() {
  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 75, motionTime, LED_GREEN, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 90, motionTime, LED_GREEN, Right_Shoulder.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Elbow.hexID, Left_Elbow.homePos + 100, motionTime, motionTime, Left_Elbow.is0601);
  Herkulex.moveOneAngle(Right_Elbow.hexID, Right_Elbow.homePos + 70, motionTime, motionTime, Right_Elbow.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 90, motionTime, LED_GREEN, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 75, motionTime, LED_GREEN, Right_Shoulder.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Elbow.hexID, Left_Elbow.homePos + 70, motionTime, LED_GREEN, Left_Elbow.is0601);
  Herkulex.moveOneAngle(Right_Elbow.hexID, Right_Elbow.homePos + 100, motionTime, LED_GREEN, Right_Elbow.is0601);
  delay(delayTime);
}