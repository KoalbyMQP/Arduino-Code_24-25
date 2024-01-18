#include <Herkulex.h>
#include <Ava.h>

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

  // Send motors to home position
  for (int i = 0; i < motorsLen; i++) {
    Herkulex.torqueON(motors[i].hexID);
    Herkulex.moveOneAngle(motors[i].hexID, motors[i].homePos, 1000, 0, motors[i].is0601);
  }

  // Rotate palm to face forward
  Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, Right_Arm_Rotator.homePos - 90, 1000, LED_GREEN, Right_Arm_Rotator.is0601);
  
  // Put left arm by side
  Herkulex.moveOneAngle(Left_Arm_Rotator.hexID, Left_Arm_Rotator.homePos + 180, 1000, LED_GREEN, Right_Arm_Rotator.is0601);
  Herkulex.moveOneAngle(Left_Arm_Abductor.hexID, Left_Arm_Abductor.homePos + 90, 1000, LED_GREEN, Right_Arm_Abductor.is0601);

  delay(1100);
}

void loop() {
  // Move right arm down
  Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, Right_Arm_Abductor.homePos + 40, 700, LED_GREEN, Right_Arm_Abductor.is0601);
  delay(650);

  // Move right arm up
  Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, Right_Arm_Abductor.homePos + 70, 700, LED_GREEN, Right_Arm_Abductor.is0601);
  delay(650);
}