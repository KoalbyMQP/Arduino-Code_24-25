#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>
#include "Ava.h"
// #include <DynamixelShield.h>
//#include <SoftwareSerial.h>
#include "set.h" //Includes set datastructure

//define this to use serial2 as output, comment out this definition to use USB
//#define DYNAMIXEL_CONTROL

//#define HUMAN_CONTROL
//#define DEBUG

#ifdef DYNAMIXEL_CONTROL
  #define SERIAL_MONITOR Serial2 //output to USB-Serial adapter
#else
  #define SERIAL_MONITOR Serial  //output to upload, USB
#endif

//Motor types
// #define HERK 0
// #define DYN  1

//List of possible numeric commands to send - cannot use 0 as a command
enum Commands { Init = 1, GetPosition = 5, SetPosition = 10,
                SetPositionT/*Set position with ID and time of motion*/ = 11,
                ArmMirror = 15, /*set arms to mirror each other*/
                SetTorque = 20,/*Set torque to "on" or "off" based on ID*/
                SetCompliant = 21,/*Set torque to "on" or "off" based on ID*/
                ReadBatteryLevel = 30,
                InitIMU = 40,
                ReadIMUData = 41,
                InitTFLuna = 50,
                ReadTFLunaData = 51,
                InitHuskyLens = 60,
                ReadHuskyLens = 61,
                SetRotationOn = 74,
                SetRotationOff = 76,
                Shutdown = 100
              };


/**
   ArduinoPoppy Class for the addition of an Arduino Mega 2560 to control Herkulex and Dynamixel motors.
*/
class ArduinoPoppy {
  public:
    //Constructor
//    ArduinoPoppy(SoftwareSerial * TFLuna);
    ArduinoPoppy();

    // Global Methods
    void Setup();
    void SetupIMU();
    Motor GetMotorByID(int motorID);
    void SetupTFLuna();
    void SetupHuskyLens();
    int ReadCommand();
    void Initialize();
    void Shutdown();
    void GetPosition();
    void SetPosition();
    void SetRotationOn();
    void SetRotationOff();
    void SetPositionT();
    void SetTorque();
    void SetCompliant();
    void ReadBatteryLevel();
    void ReadIMUData();
    void ReadHuskyLensData();
    void ReadTFLunaData();
    void PuppetMaster();
    void UpdateRobot();

    // Global Constants
    int command = 0;
//    SoftwareSerial tf;

  private:
    unsigned int compliancePWMCounter = 0;
    // Private Methods
    int getIntFromSerial();
    int getIntFromSerial(char* msg);

    //Private Objects
    // DynamixelShield dxl;
    Set compliantMotorSet;
    
    //Private Variables
    bool armMirrorModeOn = false;
    int lastMirror = 0;

    //Private Constants - This defines the robot's motor setup

    //List of motors now found in Ava.h
};

#endif
