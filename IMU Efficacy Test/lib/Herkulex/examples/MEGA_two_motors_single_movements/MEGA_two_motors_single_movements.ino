#include <Herkulex.h>

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1 
  delay(1000);
  Herkulex.reboot(0xfd); //reboot first motor
  Herkulex.reboot(2); //reboot second motor
  delay(500); 
  Herkulex.initialize(); //initialize motors
  
  Serial.println("Move Speed");
  delay(10); 
  Herkulex.moveSpeedOne(0xfd, 300, 672, 1); //move motor with 300 speed  
  Herkulex.moveSpeedOne(2, -300, 672, 1); //move motor with -300 speed
  delay(2000);
  
  Serial.println("Move Position1: 200");
  Serial.println("Move Position2: 820");
  
  delay(1);
  Herkulex.moveOne(0xfd, 200, 1500,2); //move to position 200 in 1500 milliseconds
  Herkulex.moveOne(2, 820, 500,2); //move to 820 position in 500 milliseconds

  delay(1600);
  Serial.println("");
  Serial.print("Position servo 1:"); 
  Serial.println(Herkulex.getPosition(0xfd)); //get position
  Serial.print("Position servo 2:"); 
  Serial.println(Herkulex.getPosition(2));//get position
  
  Herkulex.setLed(2,LED_PINK); //set the led 
  Herkulex.setLed(0xfd,LED_GREEN2); //set the led
  
  Herkulex.end();
}

void loop(){
}
