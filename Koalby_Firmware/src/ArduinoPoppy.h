#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h

#include <Herkulex.h>
#include "Ava.h"

// #define HUMAN_CONTROL
// #define DEBUG
#define SERIAL_MONITOR Serial // output to upload, USB

// List of possible numeric commands to send - cannot use 0 as a command
enum Commands {
    Init = 1,
    GetPosition = 5,
    SetPosition = 10,
    SetTorque = 20, /*Set torque to "on" or "off" based on ID*/
    ReadBatteryLevel = 30,
    SetRotation = 40,
    Shutdown = 100
};

/*
   ArduinoPoppy Class for the addition of an Arduino Mega 2560 to control Herkulex and Dynamixel motors.
*/
class ArduinoPoppy {
    public:
        // Constructor
        ArduinoPoppy();

        // Global Methods
        void Setup();
        Motor GetMotorByID(int motorID);
        void Initialize();
        void Shutdown();
        void GetPosition(int motorID);
        void SetPosition(int motorID, float position, int tTime);
        void SetRotation(int motorID, int goalSpeed);
        void SetTorque(int motorID, int setTorqueOn);
        void ReadBatteryLevel();
        void UpdateRobot();

        // Global Constants
        int command = 0;

    // List of motors now found in Ava.h        
};

#endif