# Arduino-Code
Arduino Side of Poppy

- HerkuleX_Readdress - set the address of a motor. NEVER run this on the real robot. It will absolutely reset every HerkuleX to the same ID and that will be 5+ hours of work
- Poppy-HerkuleX-Control - current main program, listens for messages over USB and reacts accordingly
- ReadServoAngle - Read and print angle of one HerkuleX, used to find limits

Motor Addresses

- Motor 1 - Herkulex, Right Forearm
- Motor 2 - Herkulex, Right Upper Arm
- Motor 3 - Herkulex, Right Arm Connector
- Motor 7 - Herkulex, Right Shoulder
  
- Motor B - Herkulex, Left Forearm
- Motor A - Herkulex, Right Upper Arm
- Motor C - Herkulex, Right Arm Connector
- Motor F - Herkulex, Right Shoulder

- Motor 11 - Herkulex, Torso Double Rotation Backside
- Motor 12 - Herkulex, Torso Double Rotation Frontside
- Motor 13 - Herkluex, Abdomen