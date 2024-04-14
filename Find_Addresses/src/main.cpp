#include <Herkulex.h>
#include <Ava.h>

//Checks which motors are attached and reading real values
void setup() {
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin"); 
  Herkulex.beginSerial2(115200); //open serial port 2 

  for(int i = 0; i < motorsLen; i++)
    Herkulex.reboot(motors[i].hexID); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
  Herkulex.torqueOFF(0xfe);

  Herkulex.setLed(0xfe, LED_GREEN);
}

void loop() {
  Herkulex.torqueOFF(0xfe);
  int count = 0;
  for(int i = 0; i < motorsLen; i++) {
    int angle = Herkulex.getAngle(motors[i].hexID, motors[i].is0601);
    Serial.print("Servo ");
    Serial.print(motors[i].hexID);
    Serial.print(" Angle: ");
    Serial.print(angle);
    Serial.print(" Stat: ");
    Serial.println(Herkulex.stat(motors[i].hexID));
    Herkulex.setLed(motors[i].hexID, LED_BLUE);
    if(Herkulex.stat(motors[i].hexID) == 0) count++;
  }
  Serial.println(count);

  Serial.println("Cycle");
  delay(1000);
}