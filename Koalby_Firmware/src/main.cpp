// Main firmware for Koalby humanoid robot

#include "ArduinoPoppy.h"

ArduinoPoppy robot; // Defined in ArduinoPoppy.h, motor control methods

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing");
    robot.Setup();
}

void loop() {
    robot.command = robot.ReadCommand();
#ifdef DEBUG
    if (robot.command != -1)
        Serial.println(robot.command);
#endif

    switch (robot.command) {
        case Init:
            robot.Initialize();
    #ifdef DEBUG
            Serial.println("\nINIT");
    #endif
            break;

        case InitIMU:
            robot.SetupIMU();
            break;

        case GetPosition:
            robot.GetPosition();
    #ifdef DEBUG
            Serial.println("GET POSITION");
    #endif
            break;

        case SetPosition:
            robot.SetPosition();
    #ifdef DEBUG
            Serial.println("SET POSITION");
    #endif
            break;

        case SetRotationOn:
            robot.SetRotationOn();
            break;

        case SetRotationOff:
            robot.SetRotationOff();
            break;

        case SetPositionT:
            robot.SetPositionT();
            break;

        case SetTorque:
            robot.SetTorque();
            break;

        case ReadBatteryLevel:
            robot.ReadBatteryLevel();
            break;

        case ReadIMUData:
            robot.ReadIMUData();
            break;

        case Shutdown:
            robot.Shutdown();
            break;

        default:
            break;
    }

    //  Do other tasks
    robot.UpdateRobot();
    delay(5);
}