#include <Herkulex.h>

#define HAND_LOOP true
#define TEST_WAVE false
#define HOMING false
#define FIND_BROKEN false

struct Motor{
  int hexID;
  int minPos;
  int maxPos;
  int homePos;
  int is0601;
  String description;
};

// Left Arm (Wrist to Shoulder)
Motor Left_Wrist_Abductor               = {26, -40, 130, 20, false, "Left_Wrist_Abductor"}; // Index 0
Motor Left_Elbow                        = {1, -160, 160, -70, true, "Left_Elbow"}; // Index 1
Motor Left_Arm_Rotator                  = {2, -160, 100, -100, false, "Left_Arm_Rotator"}; // Index 2
Motor Left_Arm_Abductor                 = {3, -160, 160, -64, true, "Left_Arm_Abductor"}; // Index 3

// Right Arm (Wrist to Shoulder)
Motor Right_Wrist_Abductor              = {25, -135, 35, -20, false, "Right_Wrist_Abductor"}; // Index 4
Motor Right_Elbow                       = {11, -70, 130, -40, true, "Right_Elbow"}; // Index 5
Motor Right_Arm_Rotator                 = {10, -160, 160, 20, false, "Right_Arm_Rotator"}; // Index 6
Motor Right_Arm_Abductor                = {6, -30, 80, 15, true, "Right_Arm_Abductor"};  // Index 7 Limited by 0601 wire

// Chest/Neck (Top to Bottom, Left to Right, Front to Back)
Motor Top_Neck                          = {28, 35, 110, 90, false, "Top_Neck"}; // Index 8
Motor Bottom_Neck                       = {27, -20, 160, 90, false, "Bottom_Neck"}; // Index 9
Motor Left_Shoulder                     = {7, -160, 160, -50, true, "Left_Shoulder"}; // Index 10
Motor Right_Shoulder                    = {15, -160, 90, 43, true, "Right_Shoulder"}; // Index 11
Motor Front_Chest                       = {18, -155, 160, -112, false, "Front_Chest"}; // Index 12
Motor Back_Chest                        = {17, -155, 20, -60, false, "Back_Chest"}; // Index 13

// Pelvis (Top to Bottom, Left to Right, Front to Back
Motor Hips_Rotate_Upper_Body            = {19, -160, 120, -90, false, "Hips_Rotate_Upper_Body"}; // Index 14
Motor Hips_Lean_Side_To_Side            = {21, -160, 110, 100, true, "Hips_Lean_Side_To_Side"}; // Index 15
Motor Hips_Bend_Over                    = {22, -90, 40, 25, true, "Hips_Bend_Over"}; // Index 16
Motor Left_Leg_Abductor_Front_To_Back   = {9, -20, 35, 4, false, "Left_Leg_Abductor_Front_To_Back"}; // Index 17
Motor Right_Leg_Abductor_Front_To_Back  = {8, -60, -5, -45, false, "Right_Leg_Abductor_Front_To_Back"}; // Index 18

// Left Leg (Foot to Hip)
Motor Left_Leg_Rotator                  = {14, 20, 160, -10, false, "Left_Leg_Rotator"}; // Index 22
Motor Left_Leg_Abductor_Side_To_Side    = {30, -130, 160, 30, true, "Left_Leg_Abductor_Side_To_Side"}; // Index 21
Motor Left_Knee                         = {12, -130, 0, -40, false, "Left_Knee"}; // Index 20
Motor Left_Ankle                        = {13, -25, 50, 14, false, "Left_Ankle"}; // Index 19

// Right Leg (Foot to Hip
Motor Right_Leg_Rotator                 = {4, -160, 160, 60, false, "Right_Leg_Rotator"}; // Index 23
Motor Right_Leg_Abductor_Side_To_Side   = {31, -160, 110, 0, true, "Right_Leg_Abductor_Side_To_Side"}; // Index 24
Motor Right_Knee                        = {20, -10, 120, 15, false, "Right_Knee"}; // Index 25
Motor Right_Ankle                       = {5, -40, 40, 0, false, "Right_Ankle"}; // Index 26

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
int index = 2;
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

  Herkulex.reboot(0xfe); //reboot first motor
  delay(500); 
  delay(10);
  Herkulex.initialize(); //initialize motors
  delay(10);
  Herkulex.torqueOFF(0xfe);
  delay(10);
  Herkulex.clearError(0xfe);

  if((!HOMING && !FIND_BROKEN) || TEST_WAVE || HAND_LOOP)
  {
    Serial.print("Moving Motor ");
    Serial.print(currMotor.hexID);
 
    for (Motor m : motors) {
      Herkulex.torqueON(m.hexID);
    }
  }

  // Serial.print("Moving Motor ");
  // Serial.print(currMotor.hexID);
  // Serial.print(" to ");
  // // Serial.println(currMotor.homePos);
  // Herkulex.torqueON(motorID);

  delay(1000);

  if(FIND_BROKEN)
  {
    for(int i = 0; i < 27; i++)
    {
      confirmedBroken[i] = false;
    }
  }

  Herkulex.setLed(15, LED_GREEN);
  
  delay(3000);
}

int counter = 0;
int foundAngle = 0;

void loop() {
  if(HAND_LOOP)
  {
    // left shoulder: -125 (down) to -155 (up)
    // left elbow: -31 (in) to -55 (out)
    // right shoulder: 125 (up) to 95 (down)
    // right elbow: -13 (in) to -50 (out)
    // left rotator: 131 
    // right rotator: 10
    // back chest: -5 (center), 15 (left), -25 (right)

    Herkulex.moveOneAngle(Left_Shoulder.hexID, -125, 1000, LED_GREEN, Left_Shoulder.is0601);
    Herkulex.moveOneAngle(Right_Shoulder.hexID, 125, 1000, LED_GREEN, Right_Shoulder.is0601);
    Herkulex.moveOneAngle(Left_Elbow.hexID, -31, 1000, LED_GREEN, Left_Elbow.is0601);
    Herkulex.moveOneAngle(Right_Elbow.hexID, -50, 1000, LED_GREEN, Right_Elbow.is0601);
    Herkulex.moveOneAngle(Left_Arm_Rotator.hexID, 120, 1000, LED_GREEN, Left_Arm_Rotator.is0601);
    Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, 10, 1000, LED_GREEN, Right_Arm_Rotator.is0601);
    Herkulex.moveOneAngle(Left_Arm_Abductor.hexID, 110, 1000, LED_GREEN, Left_Arm_Abductor.is0601);
    Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, -24, 1000, LED_GREEN, Right_Arm_Abductor.is0601);
    Herkulex.moveOneAngle(Left_Wrist_Abductor.hexID, -30, 1000, LED_GREEN, Left_Arm_Rotator.is0601);
    Herkulex.moveOneAngle(Right_Wrist_Abductor.hexID, -30, 1000, LED_GREEN, Right_Arm_Rotator.is0601);
    Herkulex.moveOneAngle(Front_Chest.hexID, 0, 1000, LED_GREEN, Front_Chest.is0601);
    Herkulex.moveOneAngle(Back_Chest.hexID, -5, 1000, LED_GREEN, Back_Chest.is0601);
    Herkulex.moveOneAngle(Hips_Lean_Side_To_Side.hexID, 90, 1000, LED_GREEN, Hips_Lean_Side_To_Side.is0601);
    delay(1100);

    int motionTime = 500;
    int delayTime = 350;

    while(true)
    {
      Serial.println(Herkulex.getAngle(Left_Arm_Rotator.hexID, Left_Arm_Rotator.is0601));
      Herkulex.moveOneAngle(Left_Shoulder.hexID, -155, motionTime, LED_GREEN, Left_Shoulder.is0601);
      Herkulex.moveOneAngle(Right_Shoulder.hexID, 95, motionTime, LED_GREEN, Right_Shoulder.is0601);
      // Herkulex.moveOneAngle(Back_Chest.hexID, 15, 1000, LED_GREEN, Back_Chest.is0601);
      delay(delayTime);
      Herkulex.moveOneAngle(Left_Elbow.hexID, -55, motionTime, LED_GREEN, Left_Elbow.is0601);
      Herkulex.moveOneAngle(Right_Elbow.hexID, -13, motionTime, LED_GREEN, Right_Elbow.is0601);
      delay(delayTime);
      Herkulex.moveOneAngle(Left_Shoulder.hexID, -125, motionTime, LED_GREEN, Left_Shoulder.is0601);
      Herkulex.moveOneAngle(Right_Shoulder.hexID, 125, motionTime, LED_GREEN, Right_Shoulder.is0601);
      // Herkulex.moveOneAngle(Back_Chest.hexID, 25, 1000, LED_GREEN, Back_Chest.is0601);
      delay(delayTime);
      Herkulex.moveOneAngle(Left_Elbow.hexID, -31, motionTime, LED_GREEN, Left_Elbow.is0601);
      Herkulex.moveOneAngle(Right_Elbow.hexID, -50, motionTime, LED_GREEN, Right_Elbow.is0601);
      delay(delayTime);
    }
  }

  if(TEST_WAVE)
  {
    Herkulex.moveOneAngle(Right_Shoulder.hexID, 110, 1000, LED_GREEN, Right_Shoulder.is0601);
    Herkulex.moveOneAngle(Right_Elbow.hexID, -35, 1000, LED_GREEN, Right_Elbow.is0601);
    Herkulex.moveOneAngle(Right_Wrist_Abductor.hexID, -35, 1000, LED_GREEN, Right_Wrist_Abductor.is0601);
    Herkulex.moveOneAngle(Bottom_Neck.hexID, -120, 1000, LED_GREEN, Bottom_Neck.is0601);
    delay(1100);
    while(true)
    {
      Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, -60, 500, LED_GREEN, Right_Arm_Rotator.is0601);
      Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, -40, 500, LED_GREEN, Right_Arm_Abductor.is0601);
      // Herkulex.moveOneAngle(Right_Wrist_Abductor.hexID, 0, 1000, LED_GREEN, Right_Wrist_Abductor.is0601);
      delay(500);
      Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, -110, 500, LED_GREEN, Right_Arm_Rotator.is0601);
      Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, -55, 500, LED_GREEN, Right_Arm_Abductor.is0601);
      // Herkulex.moveOneAngle(Right_Wrist_Abductor.hexID, 90, 1000, LED_GREEN, Right_Wrist_Abductor.is0601);
      delay(500);
    }
  }

  if(FIND_BROKEN)
  {
    int counter = 0;
    for(index = 0; index < 27; index++)
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

  // angle = Herkulex.getAngle(currMotor.hexID, currMotor.is0601);

  // if (off[currMotor.hexID] != 1 && (angle >= currMotor.maxPos || angle <= currMotor.minPos)) {
  //   Herkulex.torqueOFF(currMotor.hexID);
  //   Serial.print("WARNING Motor ");
  //   Serial.print(currMotor.hexID);
  //   Serial.print(" at position ");
  //   Serial.print(angle);
  //   Serial.println(" exceeded bounds and was turned off.");
  //   off[currMotor.hexID] = 1;
  // }

  // Serial.println(currMotor.hexID);
  // Serial.println(Herkulex.getAngle(currMotor.hexID, currMotor.is0601));
  // Serial.println(off[currMotor.hexID]);

  // if (Serial.available() != 0) 
  // if(false)
  // {
  //   Serial.readString();

  //   index++;
  //   currMotor = motors[index];

  //   Serial.print("Moving Motor ");
  //   Serial.print(currMotor.hexID);
  //   Serial.print(" to ");
  //   Serial.println(currMotor.homePos);
  // delay(3000);
  // }
}