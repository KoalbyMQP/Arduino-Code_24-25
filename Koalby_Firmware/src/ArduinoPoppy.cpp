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

int ArduinoPoppy::ReadCommand() {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Enter Command ");
#endif

    return getIntFromSerial();
}

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

    delay(1000);
}

void ArduinoPoppy::Shutdown() {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Robot Shutdown");
#endif

    for (Motor motor : motors) {
        Herkulex.torqueOFF(motor.hexID);
        Herkulex.setLed(motor.hexID, LED_BLUE);
    }
}

// Return position (relative to home position) in the same range as setPosition
void ArduinoPoppy::GetPosition() {
    // Get motor id
    int motorID = getIntFromSerial("Enter Motor ID");
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

void ArduinoPoppy::SetPosition() { // Set position, use 1000 for time of motion
    // Read motor number
    Motor motor = GetMotorByID(getIntFromSerial("Enter Motor Index "));

    // Read motor target position (relative to home position)
    float position = getFloatFromSerial("Enter Motor Position (float) ");

    // Send parsed command to the motor
    int mappedTarget = 0;

    // Account for home position
    position = position + motor.homePos;
    mappedTarget = min(max(position, motor.minPos), motor.maxPos);


#ifdef HUMAN_CONTROL
    // Debug print statements
    Serial.print("Moving motor ");
    Serial.print(motor.hexID);
    Serial.print(" to ");
    Serial.println(mappedTarget);
#endif

    Herkulex.torqueON(motor.hexID);
    Herkulex.moveOneAngle(motor.hexID, mappedTarget, 1000, LED_BLUE, motor.is0601); // move motor with 300 speed

#ifdef HUMAN_CONTROL
    // Debug print statements
    // delay(1100);
    Serial.print("Status: ");
    Serial.print(Herkulex.stat(motor.hexID));
    Serial.print(", Angle: ");
    Serial.println(Herkulex.getAngle(motor.hexID, motor.is0601));
#endif
}

void ArduinoPoppy::SetRotationOn() {
    int motorNum = getIntFromSerial("Enter Motor Index ");
    int goalSpeed = getIntFromSerial("Enter Motor Speed (int) ");
    Herkulex.moveSpeedOne(motors[motorNum].hexID, goalSpeed, 1000, LED_BLUE);
}

void ArduinoPoppy::SetRotationOff() {
    int motorNum = getIntFromSerial("Enter Motor Index ");
    Herkulex.moveSpeedOne(motors[motorNum].hexID, 0, 1000, LED_BLUE);
}

void ArduinoPoppy::SetPositionT() { // Set position with time of motion
    // Read motor number
    Motor motor = GetMotorByID(getIntFromSerial("Enter Motor Index "));

    // Read motor target position
    float position = getFloatFromSerial("Enter Motor Position (float) ");

    // Read time of motion
    int tTime = getIntFromSerial("Enter travel time (millis) ");

    // Send parsed command to the motor
    position = position + motor.homePos;
    int mappedTarget = min(max(position, motor.minPos), motor.maxPos);

    Herkulex.torqueON(motor.hexID);
    Herkulex.moveOneAngle(motor.hexID, mappedTarget, tTime, LED_BLUE, motor.is0601);
}

void ArduinoPoppy::SetTorque() { // Set position, use default time of motion
    // Read motor number

    Motor motor = GetMotorByID(getIntFromSerial("Enter Motor ID ")); // Read user input and hold it in a variable

    // Read motor target position
    int setTorqueOn = getIntFromSerial("Enter Motor Torque(0 = off, 1 = on) ");

    // Send parsed command to the motor
    if (setTorqueOn)
        Herkulex.torqueON(motor.hexID);
    else
        Herkulex.torqueOFF(motor.hexID);
}

void ArduinoPoppy::ReadBatteryLevel() {
    float analogValue = analogRead(A10);
    float voltage = 0.0048 * analogValue;
    SERIAL_MONITOR.println(voltage);
}

void ArduinoPoppy::UpdateRobot() {

}

// Return an integer entered over serial - options with and without a message
int ArduinoPoppy::getIntFromSerial() {
    while (SERIAL_MONITOR.available() == 0) {}
    return SERIAL_MONITOR.parseInt();
}

int ArduinoPoppy::getIntFromSerial(char *msg) {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println(msg);
#endif

    SERIAL_MONITOR.readString(); // necessary to clear serial buffer
    return getIntFromSerial();
}

float ArduinoPoppy::getFloatFromSerial() {
    while (SERIAL_MONITOR.available() == 0) {}
    return SERIAL_MONITOR.parseFloat();
}

float ArduinoPoppy::getFloatFromSerial(char *msg) {
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println(msg);
#endif

    SERIAL_MONITOR.readString(); // necessary to clear serial buffer
    return getFloatFromSerial();
}
