#include <Herkulex.h>

struct Motor{
  int hexID;
  int minPos;
  int maxPos;
  int homePos;
  int is0601;
};

// Left Arm (Wrist to Shoulder)
Motor Left_Wrist_Abductor               = {26, -40, 130, 20, false};
Motor Left_Elbow                        = {1, -90, 120, -70, true};
Motor Left_Arm_Rotator                  = {2, -160, 100, -100, false};
Motor Left_Arm_Abductor                 = {3, -160, 20, -64, false};

// Right Arm (Wrist to Shoulder)
Motor Right_Wrist_Abductor              = {25, -135, 35, -20, false};
Motor Right_Elbow                       = {11, -70, 130, -40, true};
Motor Right_Arm_Rotator                 = {10, -160, 160, 20, false};
Motor Right_Arm_Abductor                = {6, -30, 80, 15, false}; // Limited by 0601 wire

// Chest/Neck (Top to Bottom, Left to Right, Front to Back)
Motor Top_Neck                          = {28, 35, 110, 90, false};
Motor Bottom_Neck                       = {27, -20, 160, 90, false};
Motor Left_Shoulder                     = {7, -160, 160, 0, true};
Motor Right_Shoulder                    = {15, -160, 90, 43, true};
Motor Front_Chest                       = {18, -155, -65, -112, false};
Motor Back_Chest                        = {17, -155, 20, -60, false};

// Pelvis (Top to Bottom, Left to Right, Front to Back
Motor Hips_Rotate_Upper_Body            = {19, -160, 120, -90, false};
Motor Hips_Lean_Side_To_Side            = {21, -160, 110, 36, true};
Motor Hips_Bend_Over                    = {22, -90, 40, 25, true};
Motor Left_Leg_Abductor_Front_To_Back   = {9, -20, 35, 4, false};
Motor Right_Leg_Abductor_Front_To_Back  = {8, -60, -5, -45, false};

// Left Leg (Foot to Hip)
Motor Left_Leg_Rotator                  = {14, 20, 160, -10, false};
Motor Left_Leg_Abductor_Side_To_Side    = {30, -130, 160, 30, true};
Motor Left_Knee                         = {12, -130, 0, -40, false};
Motor Left_Ankle                        = {13, -25, 50, 14, false};

// Right Leg (Foot to Hip
Motor Right_Leg_Rotator                 = {4, -160, 0, 60, false};
Motor Right_Leg_Abductor_Side_To_Side   = {31, -160, 110, 0, true};
Motor Right_Knee                        = {20, -10, 120, 15, false};
Motor Right_Ankle                       = {5, -40, 40, 0, false};

Motor motors[27] = {
  Left_Wrist_Abductor, Left_Elbow, Left_Arm_Rotator, Left_Arm_Abductor, 
  Right_Wrist_Abductor, Right_Elbow, Right_Arm_Rotator, Right_Arm_Abductor, 
  Top_Neck, Bottom_Neck, Left_Shoulder, Right_Shoulder, Front_Chest, Back_Chest, 
  Hips_Rotate_Upper_Body, Hips_Lean_Side_To_Side, Hips_Bend_Over, Left_Leg_Abductor_Front_To_Back, Right_Leg_Abductor_Front_To_Back, 
  Left_Ankle, Left_Knee, Left_Leg_Abductor_Side_To_Side, Left_Leg_Rotator, 
  Right_Ankle, Right_Knee, Right_Leg_Abductor_Side_To_Side, Right_Leg_Rotator
};

int off[50];
int angle = 0;
int index = 26; // Up to 11 has been homed
Motor currMotor = motors[index];

// int counter2 = 0;
// int motorID = 1;

//Checks which motors are attached and reading real values
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Herkulex.beginSerial2(115200); //open serial port 2 
  Serial.println("Begin");

  Herkulex.reboot(0xfe); //reboot first motor
  delay(500); 
  delay(10);
  Herkulex.initialize(); //initialize motors
  delay(10);
  Herkulex.torqueOFF(0xfe);
  delay(10);
  Herkulex.clearError(0xfe);

  Serial.print("Moving Motor ");
  Serial.print(currMotor.hexID);
 
  for (Motor m : motors) {
    // Herkulex.torqueON(m.hexID);
  }

  // Serial.print("Moving Motor ");
  // Serial.print(currMotor.hexID);
  // Serial.print(" to ");
  // // Serial.println(currMotor.homePos);
  // Herkulex.torqueON(motorID);
  delay(1000);
}

int counter = 0;

void loop() {
  // THIS IS FOR MOTOR TESTING

  Serial.println(Herkulex.getAngle(currMotor.hexID, currMotor.is0601));
  counter++;
  if(counter == 1)
  {
    Serial.println(currMotor.hexID);
    counter = 0;
  }
  delay(150);
  return;
  // DELETE BEFORE THIS COMMENT ONCE ALL MOTORS ARE HOMED PROPERLY

  Herkulex.moveOneAngle(currMotor.hexID, currMotor.homePos, 1000, LED_GREEN, currMotor.is0601);

  angle = Herkulex.getAngle(currMotor.hexID, currMotor.is0601);

  if (off[currMotor.hexID] != 1 && (angle >= currMotor.maxPos || angle <= currMotor.minPos)) {
    Herkulex.torqueOFF(currMotor.hexID);
    Serial.print("WARNING Motor ");
    Serial.print(currMotor.hexID);
    Serial.print(" at position ");
    Serial.print(angle);
    Serial.println(" exceeded bounds and was turned off.");
    off[currMotor.hexID] = 1;
  }

  Serial.println(currMotor.hexID);
  Serial.println(Herkulex.getAngle(currMotor.hexID), currMotor.is0601);
  // Serial.println(off[currMotor.hexID]);

  // if (Serial.available() != 0) 
  // if(false)
  {
    Serial.readString();

    index++;
    currMotor = motors[index];

    Serial.print("Moving Motor ");
    Serial.print(currMotor.hexID);
    Serial.print(" to ");
    Serial.println(currMotor.homePos);
  delay(3000);
  }
}