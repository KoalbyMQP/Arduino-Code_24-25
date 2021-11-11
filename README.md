# Arduino-Code
Arduino Side of Poppy

- HerkuleX_Readdress - set the address of a motor. NEVER run this on the real robot. It will absolutely reset every HerkuleX to the same ID and that will be 5+ hours of work
- Poppy-HerkuleX-Control - current main program, listens for messages over USB and reacts accordingly
- ReadServoAngle - Read and print angle of one HerkuleX, used to find limits