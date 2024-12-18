# Arduino-Code_23-24
## Arduino Side of Koalby/Ava

### Included Programs
Note: For each program, there is an arduino script (\[name\]\\\[name\].ino, which can be run from the Arduino IDE as normal) and a [platformio](https://platformio.org/) project (src) for ease of use in vscode directly.

- Find_Addresses - Goes through each motor expected to be on the robot and prints its angle
- Go_Home_Pos - Combination script to find home position, go to home position, and test which motors are broken
- HerkuleX_Check - Scans all IDs for connected motors (prints any ID’s that return an non-deadzone angle)
- HerkuleX_Readdress - set the address of a HerkuleX motor. NEVER (highly not recommended) run this on the entire robot. It checks and resets all attached HerkuleX’s, so *only* the motor which you want to change should be connected
- Koalby_Firmware - Main firmware for Koalby, contains serial listen and motor control functionality. This should be left on the robot when running Python code.
  - ArduinoPoppy.c - actual robot control and functionality
  - ArduinoPoppy.h - definitions for motor parameters, contains motor array
- primities - contains code for primitive movements
  - Basic_Wave - Ava does a wave with her right arm moving up and down
  - Hand_Loop - Ava does a little dance by moving her hands in a square, similar to the robot

### Herkulex Library

Include the zip library in ./lib in the ArduinoIDE
No steps needed for using platformio in vscode

## Branches (Update as new branches are made)
### main
Ready to deploy for good production codebase on Ava's Arduino
### dev
Ready to test development codebase on Ava's Arduino
