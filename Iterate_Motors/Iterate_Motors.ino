#include <Herkulex.h>

// Checks which motors are attached and reading real values
// Goes one motor at a time, waiting for serial input to move on to the next
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1 

  for(int i = 0; i < 0xFE; i++) {
    Herkulex.reboot(i); //reboot first motor
    Herkulex.setLed(i,0);
  }

  delay(500); 
  Herkulex.initialize(); //initialize motors

  Herkulex.torqueOFF(0xFE);
  Herkulex.clearError(0xfe);
}

void loop(){
  int count = 0;
 
  for(int i = 0; i < 0xFD; i++) {
    int angle = Herkulex.getAngle(i);
    
    if(angle != -166 && angle != -83) {
      count++;
      Herkulex.torqueOFF(i);
      
      while (Serial.available() == 0) {
        Serial.print("\nGot servo ");
        Serial.print(i);
        Serial.print(" Angle: ");
        angle = Herkulex.getAngle(i);
        Serial.print(angle);
        Herkulex.setLed(i,LED_BLUE);
        delay(10);
      } //wait for data available, any input into serial monitor

      Serial.readString(); // eat input so serial is unavailable for next ID

      Herkulex.setLed(i,0);
      // Herkulex.clearError(i);
      delay(10);
    }
  }

  Serial.print("\nAll IDs scanned, found ");
  Serial.print(count);
  while (Serial.available() == 0) {}
  Serial.readString();
}