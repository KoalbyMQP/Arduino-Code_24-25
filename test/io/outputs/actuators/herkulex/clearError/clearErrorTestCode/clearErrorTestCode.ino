#include <Herkulex.h>
int n=6; //motor ID - verify your ID !!!!

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(57600);   // Open serial communications
  Serial.println("Begin"); 
  Herkulex.begin(57600,18,19);
  Herkulex.reboot(n); //reboot first motor
  delay(500); 
  Herkulex.initialize(); //initialize motors
  delay(200);  
  Herkulex.clearError(n);
  Serial.println("Clearing Motor Errors on Reboot...");
  
}

void loop(){

  Serial.println("Move Angle: -100 degrees");
  Herkulex.moveOneAngle(n, -100, 1000, LED_BLUE); //move motor with 300 speed  
  delay(1200);
  Serial.println("Move Angle: 100 degrees");
  Herkulex.moveOneAngle(n, 100, 1000, LED_GREEN); //move motor with 300 speed  
  delay(1200);
  Serial.println(Herkulex.stat(n));
  
}


