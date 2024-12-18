#include <Herkulex.h>

struct Motor{
  int hexID;
  int minPos;
  int maxPos;
  int homePos;
  int is0601;
  String description;
};

// Left Arm (Wrist to Shoulder)
Motor Left_Wrist_Abductor               = {26, -11, 147, 117, false, "Left_Wrist_Abductor"};
Motor Left_Elbow                        = {1, -125, -20, -120, true, "Left_Elbow"};
Motor Left_Arm_Rotator                  = {2, -160, 160, -90, false, "Left_Arm_Rotator"};
Motor Left_Arm_Abductor                 = {3, -120, 140, 35, true, "Left_Arm_Abductor"}; // limited by wire

// Right Arm (Wrist to Shoulder)
Motor Right_Wrist_Abductor              = {25, -2, 155, 27, false, "Right_Wrist_Abductor"};
Motor Right_Elbow                       = {11, -116, -10, -110, true, "Right_Elbow"};
Motor Right_Arm_Rotator                 = {10, -160, 160, 90, false, "Right_Arm_Rotator"};
Motor Right_Arm_Abductor                = {6, -73, 136, 32, true, "Right_Arm_Abductor"};

// Chest/Neck (Top to Bottom, Left to Right, Front to Back)
Motor Top_Neck                          = {28, -103, -33, -83, false, "Top_Neck"};
Motor Bottom_Neck                       = {27, -160, 100, -90, false, "Bottom_Neck"};
Motor Left_Shoulder                     = {7, -160, 145, -55, true, "Left_Shoulder"};
Motor Right_Shoulder                    = {15, -160, 160, -79, true, "Right_Shoulder"};
Motor Front_Chest                       = {18, -30, 43, 0, false, "Front_Chest"};
Motor Back_Chest                        = {17, -70, 55, -5, false, "Back_Chest"};

// Pelvis (Top to Bottom, Left to Right, Front to Back
Motor Hips_Rotate_Upper_Body            = {19, -140, 160, 15, false, "Hips_Rotate_Upper_Body"};
Motor Hips_Lean_Side_To_Side            = {21, 25, 160, 107, true, "Hips_Lean_Side_To_Side"};
Motor Hips_Bend_Over                    = {22, -125, -67, -78, true, "Hips_Bend_Over"};
Motor Left_Leg_Abductor_Side_To_Side    = {9, 61, 100, 78, false, "Left_Leg_Abductor_Side_To_Side"};
Motor Right_Leg_Abductor_Side_To_Side   = {8, -14, 16, -2, false, "Right_Leg_Abductor_Side_To_Side"};

// Left Leg (Foot to Hip)
Motor Left_Leg_Rotator                  = {14, -20, 160, 3, false, "Left_Leg_Rotator"};
Motor Left_Leg_Abductor_Front_To_Back   = {30, 19, 155, 122, true, "Left_Leg_Abductor_Front_To_Back"};
Motor Left_Knee                         = {12, -125, -8, -3, false, "Left_Knee"};
Motor Left_Ankle                        = {13, -75, -5, -38, false, "Left_Ankle"};

// Right Leg (Foot to Hip
Motor Right_Leg_Rotator                 = {4, -120, 160, 0, false, "Right_Leg_Rotator"};
Motor Right_Leg_Abductor_Front_To_Back  = {31, -160, 11, -89, true, "Right_Leg_Abductor_Front_To_Back"};
Motor Right_Knee                        = {20, -105, 18, -100, false, "Right_Knee"};
Motor Right_Ankle                       = {5, -35, 40, 0, false, "Right_Ankle"};

int motorsLen = 27;
Motor motors[27] = {
  Left_Elbow, Left_Arm_Rotator, Left_Arm_Abductor, Right_Leg_Rotator, Right_Ankle, // 1-5
  Right_Arm_Abductor, Left_Shoulder, Right_Leg_Abductor_Side_To_Side, Left_Leg_Abductor_Side_To_Side, Right_Arm_Rotator, // 6-10
  Right_Elbow, Left_Knee, Left_Ankle, Left_Leg_Rotator, Right_Shoulder, // 11-15
  Back_Chest, Front_Chest, Hips_Rotate_Upper_Body, Right_Knee, Hips_Lean_Side_To_Side, // 17 - 21
  Hips_Bend_Over, Right_Wrist_Abductor, Left_Wrist_Abductor, Bottom_Neck, Top_Neck, // 22, 25-28
  Left_Leg_Abductor_Front_To_Back, Right_Leg_Abductor_Front_To_Back // 30-31
};

int motionTime = 400;
int delayTime = 350;

//Checks which motors are attached and reading real values
void setup()  
{
  delay(100);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial3(115200); //open serial port 1  

  for(int i =0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
 
  Herkulex.torqueON(0xfe);
  delay(100);

  // non moving
  for (int i = 0; i < motorsLen; i++) {
    Herkulex.torqueON(motors[i].hexID);
    Herkulex.moveOneAngle(motors[i].hexID, motors[i].homePos, 1000, 0, motors[i].is0601);
  }

  // Move arms down and forward a little
  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 75, 1000, 0, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 75, 1000, 0, Right_Shoulder.is0601);
  Herkulex.moveOneAngle(Left_Arm_Abductor.hexID, Left_Arm_Abductor.homePos + 70, 1000, 0, Left_Arm_Abductor.is0601);
  Herkulex.moveOneAngle(Right_Arm_Abductor.hexID, Right_Arm_Abductor.homePos - 70, 1000, 0, Right_Arm_Abductor.is0601);
  Herkulex.moveOneAngle(Left_Arm_Rotator.hexID, Left_Arm_Rotator.homePos + 180, 1000, 0, Left_Arm_Rotator.is0601);
  Herkulex.moveOneAngle(Right_Arm_Rotator.hexID, Right_Arm_Rotator.homePos - 180, 1000, 0, Right_Arm_Rotator.is0601);

  delay(1100);
}

// angles and timing not tuned well :)
void loop() {
  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 75, motionTime, LED_GREEN, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 90, motionTime, LED_GREEN, Right_Shoulder.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Elbow.hexID, Left_Elbow.homePos + 100, motionTime, motionTime, Left_Elbow.is0601);
  Herkulex.moveOneAngle(Right_Elbow.hexID, Right_Elbow.homePos + 70, motionTime, motionTime, Right_Elbow.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Shoulder.hexID, Left_Shoulder.homePos - 90, motionTime, LED_GREEN, Left_Shoulder.is0601);
  Herkulex.moveOneAngle(Right_Shoulder.hexID, Right_Shoulder.homePos + 75, motionTime, LED_GREEN, Right_Shoulder.is0601);
  delay(delayTime);

  Herkulex.moveOneAngle(Left_Elbow.hexID, Left_Elbow.homePos + 70, motionTime, LED_GREEN, Left_Elbow.is0601);
  Herkulex.moveOneAngle(Right_Elbow.hexID, Right_Elbow.homePos + 100, motionTime, LED_GREEN, Right_Elbow.is0601);
  delay(delayTime);
}