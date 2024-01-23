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
    SetPositionT /*Set position with ID and time of motion*/ = 11,
    SetTorque = 20, /*Set torque to "on" or "off" based on ID*/
    ReadBatteryLevel = 30,
    InitIMU = 40,
    ReadIMUData = 41,
    SetRotationOn = 74,
    SetRotationOff = 76,
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
        int ReadCommand();
        void Initialize();
        void Shutdown();
        void GetPosition();
        void SetPosition();
        void SetRotationOn();
        void SetRotationOff();
        void SetPositionT();
        void SetTorque();
        void ReadBatteryLevel();
        void ReadIMUData();
        void UpdateRobot();

        // Global Constants
        int command = 0;

    private:
        // Private Methods
        int getIntFromSerial();
        int getIntFromSerial(char *msg);
        float getFloatFromSerial();
        float getFloatFromSerial(char *msg);

        // List of motors now found in Ava.h
};

#endif