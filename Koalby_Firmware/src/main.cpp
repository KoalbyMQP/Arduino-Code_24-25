// Main firmware for Koalby humanoid robot

#include "ArduinoPoppy.h"

#define CHECK_MOTOR_STATUSES true

ArduinoPoppy robot; // Defined in ArduinoPoppy.h, motor control methods

int getFirstInt(String s, int startIndex=0) {
    return s.substring(startIndex, s.indexOf(" ", startIndex)).toInt();
}

float getFirstFloat(String s, int startIndex=0) {
    return s.substring(startIndex, s.indexOf(" ", startIndex)).toFloat();
}

String removeFirstArg(String s) {
    return s.substring(s.indexOf(" ") + 1);
}

void setup() {
    Serial.begin(115200);
#ifdef HUMAN_CONTROL
    Serial.println("Initializing");
#endif
    robot.Setup();
}

int motorID = 0;
float position = 0;
int speed = 0;
int tTime = 0;
int torqueOn = 0;
bool initOnStartup = false;

void loop() {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Enter Command ");
#endif
    while (!Serial.available() && !initOnStartup) {
        if(CHECK_MOTOR_STATUSES) {
            robot.CheckMotorStatuses();
            delay(2000);
        }
    } // wait until command recieved
    String cmd = SERIAL_MONITOR.readStringUntil('\n');
#ifdef DEBUG
        Serial.print("Full command: ");
        Serial.println(cmd);
#endif
    int hasArgs = cmd.indexOf(" ") > -1;

    if (hasArgs) {
        robot.command = getFirstInt(cmd);
        cmd = removeFirstArg(cmd);
    } else {
        robot.command = cmd.toInt();
    }

    if(initOnStartup)
    {
        initOnStartup = false;
        robot.command = Init;
    }

#ifdef DEBUG
    if (robot.command != -1)
        Serial.println(robot.command);
        Serial.println(cmd);
#endif

    switch (robot.command) {
        case Init:
            robot.Initialize();
            #ifdef DEBUG
            Serial.println("\nINIT");
            #endif
            break;

        case GetPosition:
            motorID = cmd.toInt();

            robot.GetPosition(motorID);
            #ifdef DEBUG
            Serial.println("GET POSITION");
            Serial.println("Args:");
            Serial.println(motorID);
            #endif
            break;

        case SetPosition:
            motorID = getFirstInt(cmd);
            cmd = removeFirstArg(cmd);

            position = getFirstFloat(cmd);
            tTime = cmd.substring(cmd.indexOf(" ") + 1).toInt();

            robot.SetPosition(motorID, position, tTime);
            #ifdef DEBUG
            Serial.println("SET POSITION");
            Serial.println("Args:");
            Serial.println(motorID);
            Serial.println(position);
            Serial.println(tTime);
            #endif
            break;

        case SetTorque:
            motorID = getFirstInt(cmd);
            torqueOn = cmd.substring(cmd.indexOf(" ") + 1).toInt();

            robot.SetTorque(motorID, torqueOn);
            #ifdef DEBUG
            Serial.println("SET TORQUE");
            Serial.println("Args:");
            Serial.println(motorID);
            Serial.println(torqueOn);
            #endif
            break;

        case SetRotation:
            motorID = getFirstInt(cmd);
            speed = cmd.substring(cmd.indexOf(" ") + 1).toInt();

            robot.SetRotation(motorID, speed);
            break;

        case ReadBatteryLevel:
            robot.ReadBatteryLevel();
            break;

        case Shutdown:
            robot.Shutdown();
            break;

        case CheckMotors:
            robot.CheckMotorStatuses();
            break;

        default:
            break;
    }

    //  Do other tasks
    robot.UpdateRobot();
    delay(5);
}