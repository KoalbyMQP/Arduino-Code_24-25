#include <Herkulex.h>

//NEVER RUN THIS PROGRAM ON THE ACTUAL ROBOT IT WILL RESET EVERYTHING
int oldID = 253; //set the motor ID - cant be FE
int newID = 13; //set the motor ID
bool is0601 = false;

void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial2(115200); //open serial port 2
  Herkulex.reboot(0xfe); //reboot 
  delay(500); 
  Herkulex.initialize(); //initialize motors
  delay(500);
  Herkulex.torqueOFF(0xfe);
  delay(10);

  Herkulex.set_ID(oldID, newID);
  delay(10);
    
  for(int i = 0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors

  Herkulex.setLed(0xfe, LED_BLUE);
  delay(10);
  Herkulex.setLed(newID, LED_GREEN);
  Serial.println("Done changing id");
}

void loop() {
   Serial.println(Herkulex.stat(newID));
}
