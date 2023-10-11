#include <Herkulex.h>

//Checks which motors are attached and reading real values
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin"); 
  Herkulex.beginSerial2(115200); //open serial port 2 

  for(int i =0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
  Herkulex.torqueOFF(0xfe);
  Herkulex.setLed(0xfe, LED_GREEN);
}

void loop(){
  int count = 0;

  for(int i =0;i<0xFD;i++){
    int angle = Herkulex.getAngle(i, false); // assume 0201, does not matter for finding connected motors
    if(angle != -166 && angle != -83){
      Serial.print("Got servo ");
      Serial.print(i);
      Serial.print(" Angle: ");
      Serial.println(angle);
      Herkulex.setLed(i,LED_BLUE);
      Herkulex.torqueOFF(i);
      count++;
    }
  }
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" motors.");
  Serial.println("Cycle");
  delay(1000);
}