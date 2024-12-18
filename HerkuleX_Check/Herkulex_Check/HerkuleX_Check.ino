#include <Herkulex.h>

//Checks which motors are attached and reading real values
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial3(115200); //open serial port 1  

  for(int i =0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
  Herkulex.torqueOFF(0xfe);
  Herkulex.clearError(0xfe);
  Herkulex.setLed(0xfe, LED_GREEN);
}

void loop(){
  int count;
  for(int i =0;i<0xFE;i++){
    int angle = Herkulex.getAngle(i, false); // Assume 0201, just see if the motor is hooked up
    if(angle != -166){
      Serial.print("Got servo ");
      Serial.print(i);
      Serial.print(" Angle: ");
      Serial.println(angle);
      // Herkulex.clearError(i);
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
