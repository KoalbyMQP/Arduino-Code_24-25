#include "Ava.h"

// Left Arm (Wrist to Shoulder)
Motor Left_Wrist_Abductor = {26, -160, 150, -59, false, "Left_Wrist_Abductor"};
Motor Left_Elbow = {1, -160, 160, -3, true, "Left_Elbow"};
Motor Left_Arm_Rotator = {2, -160, 160, 90, false, "Left_Arm_Rotator"};
Motor Left_Arm_Abductor = {3, -200, 160, -106, true, "Left_Arm_Abductor"}; // limited by wire

// Right Arm (Wrist to Shoulder)
Motor Right_Wrist_Abductor = {25, -160, 160, 55, false, "Right_Wrist_Abductor"};
Motor Right_Elbow = {11, -160, 160, -115, true, "Right_Elbow"};
Motor Right_Arm_Rotator = {10, -160, 160, -21, false, "Right_Arm_Rotator"}; 
Motor Right_Arm_Abductor = {6, -160, 160, -19, true, "Right_Arm_Abductor"};

// Chest/Neck (Top to Bottom, Left to Right, Front to Back)
Motor Top_Neck = {28, -160, 160, 48, false, "Top_Neck"};
Motor Bottom_Neck = {27, -160, 160, -96, false, "Bottom_Neck"};
Motor Left_Shoulder = {7, -160, 160, -6, true, "Left_Shoulder"};
Motor Right_Shoulder = {15, -160, 160, 54, true, "Right_Shoulder"};
Motor Front_Chest = {18, -160, 160, -1, false, "Front_Chest"};
Motor Back_Chest = {17, -160, 160, -62, false, "Back_Chest"};

// Pelvis (Top to Bottom, Left to Right, Front to Back
Motor Hips_Rotate_Upper_Body = {19, -160, 160, -13, false, "Hips_Rotate_Upper_Body"}; 
Motor Hips_Lean_Side_To_Side = {21, -160, 160, 29, true, "Hips_Lean_Side_To_Side"};
Motor Hips_Bend_Over = {22, -38, 50, 26, true, "Hips_Bend_Over"};
Motor Left_Leg_Abductor_Side_To_Side = {8, -160, 250, 166, false, "Left_Leg_Abductor_Side_To_Side"}; 
Motor Right_Leg_Abductor_Side_To_Side = {9, -160, 160, -50, false, "Right_Leg_Abductor_Side_To_Side"};

// Left Leg (Foot to Hip)
Motor Left_Leg_Rotator = {14, -160, 160, -107, false, "Left_Leg_Rotator"};
Motor Left_Leg_Abductor_Front_To_Back = {30, -318, 160, 59, true, "Left_Leg_Abductor_Front_To_Back"};
Motor Left_Knee = {12, -160, 160, -20, true, "Left_Knee"};
Motor Left_Ankle = {13, -160, 160, 83, false, "Left_Ankle"};

// Right Leg (Foot to Hip)
Motor Right_Leg_Rotator = {4, -160, 160, 8, false, "Right_Leg_Rotator"};
Motor Right_Leg_Abductor_Front_To_Back = {31, -288, 160, -98, true, "Right_Leg_Abductor_Front_To_Back"};
Motor Right_Knee = {20, -160, 160, -6, true, "Right_Knee"};
Motor Right_Ankle = {5, -160, 160, 57, false, "Right_Ankle"};

// Ordered in motors array by id (first value of struct)
// FYI ids 16, 23-24, and 29 are unused
int motorsLen = MOTOR_COUNT;
Motor motors[MOTOR_COUNT] = {
    Left_Elbow, Left_Arm_Rotator, Left_Arm_Abductor, Right_Leg_Rotator, Right_Ankle,                                       // 1-5
    Right_Arm_Abductor, Left_Shoulder, Left_Leg_Abductor_Side_To_Side, Right_Leg_Abductor_Side_To_Side, Right_Arm_Rotator, // 6-10
    Right_Elbow, Left_Knee, Left_Ankle, Left_Leg_Rotator, Right_Shoulder,                                                  // 11-15
    Back_Chest, Front_Chest, Hips_Rotate_Upper_Body, Right_Knee, Hips_Lean_Side_To_Side,                                   // 17 - 21
    Hips_Bend_Over, Right_Wrist_Abductor, Left_Wrist_Abductor, Bottom_Neck, Top_Neck,                                      // 22, 25-28
    Left_Leg_Abductor_Front_To_Back, Right_Leg_Abductor_Front_To_Back                                                      // 30-31
};