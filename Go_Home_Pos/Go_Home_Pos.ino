#include <Herkulex.h>

struct Motor{
  int hexID;
  int minPos;
  int maxPos;
  int homePos;
};

// Left Arm (Wrist to Shoulder)
Motor Left_Wrist_Abductor               = {26, -130, 40, -10};
Motor Left_Elbow                        = {1, -90, 120, -46};
Motor Left_Arm_Rotator                  = {2, -15, 160, 90};
Motor Left_Arm_Abductor                 = {3, -150, 20, 0};

// Right Arm (Wrist to Shoulder)
Motor Right_Wrist_Abductor              = {25, -130, 40, -10};
Motor Right_Elbow                       = {11, -70, 130, -40};
Motor Right_Arm_Rotator                 = {10, -105, 160, 25};
Motor Right_Arm_Abductor                = {6, -20, 50, 0}; // Severely limited by 0601 wire

// Chest/Neck (Top to Bottom, Left to Right, Front to Back)
Motor Top_Neck                          = {28, 30, 120, 90};
Motor Bottom_Neck                       = {27, 0, 160, 90};
Motor Left_Shoulder                     = {7, -150, 160, -5};
Motor Right_Shoulder                    = {15, -160, 40, -130};
Motor Front_Chest                       = {18, -150, -66, -112};
Motor Back_Chest                        = {17, -160, 40, -60};

// Pelvis (Top to Bottom, Left to Right, Front to Back
Motor Hips_Rotate_Upper_Body            = {19, -160, 0, 145};
Motor Hips_Lean_Side_To_Side            = {21, -10, 160, -120};
Motor Hips_Bend_Over                    = {22, -90, 40, 30};
Motor Left_Leg_Abductor_Front_To_Back   = {9, -15, 30, 7};
Motor Right_Leg_Abductor_Front_To_Back  = {8, -55, -15, -25};

// Left Leg (Foot to Hip)
Motor Left_Leg_Rotator                  = {14, 110, 160, 150};
Motor Left_Leg_Abductor_Side_To_Side    = {30, -130, 160, 90};
Motor Left_Knee                         = {12, -130, 0, -40};
Motor Left_Ankle                        = {13, -25, 50, 30};

// Right Leg (Foot to Hip
Motor Right_Leg_Rotator                 = {4, -160, 0, -150};
Motor Right_Leg_Abductor_Side_To_Side   = {31, -160, 115, -90};
Motor Right_Knee                        = {20, -10, 120, 36};
Motor Right_Ankle                       = {5, -40, 40, -30};

Motor motors[27] = {
  Left_Wrist_Abductor, Left_Elbow, Left_Arm_Rotator, Left_Arm_Abductor, 
  Right_Wrist_Abductor, Right_Elbow, Right_Arm_Rotator, Right_Arm_Abductor, 
  Top_Neck, Bottom_Neck, Left_Shoulder, Right_Shoulder, Front_Chest, Back_Chest, 
  Hips_Rotate_Upper_Body, Hips_Lean_Side_To_Side, Hips_Bend_Over, Left_Leg_Abductor_Front_To_Back, Right_Leg_Abductor_Front_To_Back, 
  Left_Ankle, Left_Knee, Left_Leg_Abductor_Side_To_Side, Left_Leg_Rotator, 
  Right_Ankle, Right_Knee, Right_Leg_Abductor_Side_To_Side, Right_Leg_Rotator
};

int off[50];

//Checks which motors are attached and reading real values
void setup()  
{
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(115200);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1 
//  Herkulex.beginSerial2(115200); //open serial port 2 

  for(int i =0;i<0xFE;i++)
    Herkulex.reboot(i); //reboot first motor

  delay(500); 
  Herkulex.initialize(); //initialize motors
  Herkulex.torqueOFF(0xfe);
 
  for (Motor m : motors) {
    Herkulex.torqueON(m.hexID);
  }
}

void loop(){
  for (Motor m : motors) {
    int angle = Herkulex.getAngle(m.hexID);

    if (off[m.hexID] != 1 && (angle >= m.maxPos || angle <= m.minPos)) {
      Herkulex.torqueOFF(m.hexID);
      Serial.print("WARNING Motor ");
      Serial.print(m.hexID);
      Serial.print(" at position ");
      Serial.print(angle);
      Serial.println(" exceeded bounds and was turned off.");
      off[m.hexID] = 1;
    }
  }

  delay(10);
}