#include <Herkulex.h>
#include <Ava.h>

#define HOMING false
#define FIND_BROKEN false
#define GO_HOME false

int off[50];
int angle = 0;
int index = 0;
Motor currMotor = motors[index];
bool confirmedBroken[27];

// int counter2 = 0;
// int motorID = 1;

//Checks which motors are attached and reading real values
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Herkulex.beginSerial3(115200); //open serial port 3
  Serial.println("\nBegin");

  // for(int i = 0; i < 27; i++)
  // {
  //   int registerNum = 12;
  //   if(!motors[i].is0601)
  //   {
  //     Herkulex.writeRegistryEEP(motors[i].hexID, registerNum, 10);
  //   }

  // }

  // Herkulex.writeRegistryEEP(26, 12, 92);

  Herkulex.reboot(0xfe); //reboot motors
  delay(500);
  Herkulex.initialize(); //initialize motors
  delay(10);
  Herkulex.torqueOFF(0xfe);
  delay(10);
  Herkulex.clearError(0xfe);

  for (Motor m : motors) {
    Herkulex.torqueOFF(m.hexID);
  }

  delay(1000);

  if(FIND_BROKEN)
  {
    for(int i = 0; i < 27; i++)
    {
      confirmedBroken[i] = false;
    }
  }

  // Send motors to home position
  // aka make t-pose
  if (GO_HOME) {
      for (int i = 0; i < motorsLen; i++) {
        Herkulex.torqueON(motors[i].hexID);
        Herkulex.moveOneAngle(motors[i].hexID, motors[i].homePos, 1000, LED_GREEN, motors[i].is0601);
      }
      delay(4000);
  }
}

int counter = 0;
int foundAngle = 0;

void loop() {
  if(FIND_BROKEN)
  {
    int counter = 0;
    for(index = 0; index < motorsLen; index++)
    {
      currMotor = motors[index];
      foundAngle = Herkulex.getAngle(currMotor.hexID, currMotor.is0601);
      if((foundAngle < -160 && !confirmedBroken[index]))
      {
        Herkulex.setLed(currMotor.hexID, LED_RED);
        confirmedBroken[index] = true;
        Serial.print(currMotor.hexID);
        Serial.print(", DRS-");
        Serial.print(currMotor.is0601 ? "0601" : "0201");
        Serial.print(", " + currMotor.description);
        Serial.print(", Angle: ");
        Serial.println(Herkulex.getAngle(currMotor.hexID, currMotor.is0601));
        Serial.print("Error number: ");
        Serial.println(Herkulex.stat(currMotor.hexID));
        counter++;
      }
      else if(!confirmedBroken[index] || foundAngle >= -160)
      {
        Herkulex.setLed(currMotor.hexID, LED_GREEN);
        confirmedBroken[index] = false;
      }
      delay(10);
    }
    if(counter > 0)
    {
      Serial.print(counter);
      Serial.println(" motors broken");
    }
    delay(2000);
    return;
  }

  // For motor home position testing
  if(HOMING)
  {
    Serial.println(Herkulex.getAngle(currMotor.hexID, currMotor.is0601));
    Serial.println(Herkulex.stat(currMotor.hexID));
    counter++;
    if(counter == 1)
    {
      Serial.println(currMotor.hexID);
      counter = 0;
    }
    delay(150);
    return;
  }

  // Herkulex.moveOneAngle(currMotor.hexID, currMotor.homePos, 1000, LED_GREEN, currMotor.is0601);

  Herkulex.setLed(currMotor.hexID, LED_RED);
  angle = Herkulex.getAngle(currMotor.hexID, currMotor.is0601);

  Serial.print(currMotor.hexID);
  Serial.print(", ");
  Serial.print(currMotor.description);
  Serial.print(", ");
  Serial.print(angle);
  Serial.print(", ");
  Serial.println(Herkulex.stat(currMotor.hexID));
  Herkulex.clearError(0xfe);

  if (Serial.available() != 0) 
  {
    Serial.readString();
    Herkulex.setLed(currMotor.hexID, LED_GREEN);
    // Herkulex.torqueON(currMotor.hexID);

    index++;

    if (index == motorsLen)
      index = 0;

    currMotor = motors[index];

    delay(1500);
  }
}