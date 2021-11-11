//Disable torque for a specific motor and constantly print its position, used to figure out limits and home position
#include <Herkulex.h>
int n=0x11; //motor ID - verify your ID !!!!

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1 
  Herkulex.reboot(n); //reboot first motor
  delay(500); 
  Herkulex.initialize(); //initialize motors
  delay(200);  
}

void loop(){
  Herkulex.torqueOFF(n);
  Serial.print("Get servo Angle:");
  Serial.println(Herkulex.getAngle(n));
  //Herkulex.moveOneAngle(n, 100, 1000, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  
}
