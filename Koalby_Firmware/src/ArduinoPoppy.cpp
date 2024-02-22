#include "ArduinoPoppy.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_PERIOD_MS 10

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

uint8_t motorIndices[254]; // the index in the motors array of each motor ID

ArduinoPoppy::ArduinoPoppy() {
}

void ArduinoPoppy::Setup() {
    delay(2000);                  // a delay to have time for serial monitor opening
    SERIAL_MONITOR.begin(115200); // Open serial communications
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Begin");
#endif

    // Start HerkuleX
    Herkulex.beginSerial3(115200); // open serial port 3 for HerkuleX
    delay(100);
    Herkulex.reboot(0xfe); //reboot first motor

    delay(10); 
    Herkulex.initialize(); //initialize motors

    // Initializes motorIndices array (for GetMotorByID)
    for (int i = 0; i < 254; i++) {
        motorIndices[i] = -1;
    }
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorIndices[motors[i].hexID] = i;
    }
}

// Returns the motor with the given hexID
Motor ArduinoPoppy::GetMotorByID(int motorID) {
    return motors[motorIndices[motorID]];
}

// command structure: '1'
void ArduinoPoppy::Initialize() {
    // initialize motors
    Herkulex.initialize();
    Herkulex.reboot(0xfe);
    delay(500);

    // Move each motor to home position (T-pose)
    for (Motor motor : motors) {
        Herkulex.torqueON(motor.hexID);
        Herkulex.moveOneAngle(motor.hexID, motor.homePos, 1000, LED_GREEN, motor.is0601);
    }

    delay(1500);
}

// Return position (relative to home position) in the same range as setPosition
// command structure: '5 id'
// i.e. '5 5'
void ArduinoPoppy::GetPosition(int motorID) {
    Motor motor = GetMotorByID(motorID);

#ifdef HUMAN_CONTROL
    // Debug print statements
    Serial.print("Getting position from motor ");
    Serial.println(motor.hexID);
    Serial.print("Angle: ");
    Serial.println(Herkulex.getAngle(motor.hexID, motor.is0601));
#endif

    float angle = 0;
    int attempt = 0;

    do { // attempt communication 10 times in case of loose connections
        angle = Herkulex.getAngle(motor.hexID, motor.is0601);
        attempt++;
    } while (angle < -164 && attempt < 10);

    SERIAL_MONITOR.println(angle - motor.homePos);
}

// command structure: '10 id pos time'
// i.e. '10 5 0.0 1000'
void ArduinoPoppy::SetPosition(int motorID, float position, int tTime) { // Set position with time of motion
    // Read motor number
    Motor motor = GetMotorByID(motorID);

    // Send parsed command to the motor
    position = position + motor.homePos;
    int mappedTarget = min(max(position, motor.minPos), motor.maxPos);

    Herkulex.torqueON(motor.hexID);
    Herkulex.moveOneAngle(motor.hexID, mappedTarget, tTime, LED_BLUE, motor.is0601);
}

// command structure: '20 id turnOn'
// i.e. '20 5 1'
void ArduinoPoppy::SetTorque(int motorID, int setTorqueOn) { // Set position, use default time of motion
    // Read motor number

    Motor motor = GetMotorByID(motorID); // Read user input and hold it in a variable

    // Send parsed command to the motor
    if (setTorqueOn)
        Herkulex.torqueON(motor.hexID);
    else
        Herkulex.torqueOFF(motor.hexID);
}

// command structure: '30'
void ArduinoPoppy::ReadBatteryLevel() {
    float analogValue = analogRead(A10);
    float voltage = 0.0048 * analogValue;
    SERIAL_MONITOR.println(voltage);
}

// command structure: '40 id speed'
// i.e. '40 5 0'
void ArduinoPoppy::SetRotation(int motorID, int goalSpeed) {
    Motor motor = GetMotorByID(motorID);
    Herkulex.moveSpeedOne(motor.hexID, goalSpeed, 1000, LED_BLUE);
}

// command structure: '100'
void ArduinoPoppy::Shutdown() {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Robot Shutdown");
#endif

    for (Motor motor : motors) {
        Herkulex.torqueOFF(motor.hexID);
        Herkulex.setLed(motor.hexID, LED_BLUE);
    }
}

void ArduinoPoppy::UpdateRobot() {
}

void ArduinoPoppy::CheckMotorStatuses()
{
    for (Motor motor : motors)
    {
        byte stat = Herkulex.stat(motor.hexID);
        if (stat != 0)
        {
            Serial.print(motor.hexID);
            Serial.print(" ");
            Serial.print(stat);
            Serial.print(" ");
            Serial.println(Herkulex.getAngle(motor.hexID, motor.is0601));
        }
    }
    Serial.println("END");
}