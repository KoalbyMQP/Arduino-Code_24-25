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
    Herkulex.beginSerial3(115200); // open serial port 1 for HerkuleX's
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

void ArduinoPoppy::SetupIMU() {
}

int ArduinoPoppy::ReadCommand() {
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

// Return position in the same range as setPosition
void ArduinoPoppy::GetPosition() {
    // Get motor id
    int motorID = getIntFromSerial("Enter Motor Id");
    Motor motor = GetMotorByID(motorID);

    // Debug print statements

    // Serial.print("Getting position from motor ");
    // Serial.println(motor.hexID);
    // Serial.print("Angle: ");
    // Serial.println(Herkulex.getAngle(motor.hexID, motor.is0601));

    float angle = 0;
    int attempt = 0;

    do {
        angle = Herkulex.getAngle(motor.hexID, motor.is0601);
        attempt++;
    } while (angle < -164 && attempt < 10);

    SERIAL_MONITOR.println(angle - motor.homePos);
}

void ArduinoPoppy::SetPosition() { // Set position, use 1000 for time of motion
    // Read motor number
    int motorID = getIntFromSerial("Enter Motor Index ");

    // Read motor target position
    float position = getFloatFromSerial("Enter Motor Position (float) ");

    // Send parsed command to the motor
    int mappedTarget = 0;
    Motor motor = GetMotorByID(motorID);

    // Account for home position
    position = position + motor.homePos;
    mappedTarget = min(max(position, motor.minPos), motor.maxPos);

    // Debug print statements

    // Serial.print("Moving motor ");
    // Serial.print(motor.hexID);
    // Serial.print(" to ");
    // Serial.println(mappedTarget);
    // Serial.print("Status: ");
    // Serial.print(Herkulex.stat(motor.hexID));
    // Serial.print(", Angle: ");
    // Serial.println(Herkulex.getAngle(motor.hexID, motor.is0601));

    Herkulex.moveOneAngle(motor.hexID, mappedTarget, 1000, LED_BLUE, motor.is0601); // move motor with 300 speed
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
    int motorNum = getIntFromSerial("Enter Motor Index ");

    // Read motor target position
    float position = getFloatFromSerial("Enter Motor Position (float) ");

    // Read time of motion
    int tTime = getIntFromSerial("Enter travel time (millis) ");

    // Send parsed command to the motor
    int mappedTarget = 0;
    // Account for motor direction when setting limits
    if (position, motors[motorNum].minPos < motors[motorNum].maxPos) {
        position += motors[motorNum].homePos;
        mappedTarget = min(max(position, motors[motorNum].minPos), motors[motorNum].maxPos);
        /*SERIAL_MONITOR.print("Val: ");
          SERIAL_MONITOR.print(position);
          SERIAL_MONITOR.print("actual: ");
          SERIAL_MONITOR.println(mappedTarget);*/
    } else {
        position = -position + motors[motorNum].homePos;
        mappedTarget = max(min(position, motors[motorNum].minPos), motors[motorNum].maxPos);
        /*SERIAL_MONITOR.print("Val2: ");
          SERIAL_MONITOR.print(position);
          SERIAL_MONITOR.print("actual: ");
          SERIAL_MONITOR.println(mappedTarget);*/
    }

    Herkulex.moveOneAngle(motors[motorNum].hexID, mappedTarget, tTime, LED_BLUE, motors[motorNum].is0601); // move motor with 300 speed
}

void ArduinoPoppy::SetTorque() { // Set position, use default time of motion
    // Read motor number
#ifdef HUMAN_CONTROL
    SERIAL_MONITOR.println("Enter Motor Index "); // Prompt User for input
#endif

    while (SERIAL_MONITOR.available() == 0) {} // wait for user input
    int motorNum = SERIAL_MONITOR.parseInt(); // Read user input and hold it in a variable

    // Read motor target position
    int setTorqueOn = getIntFromSerial("Enter Motor Torque(0 = off, 1 = on) ");

    // Send parsed command to the motor
    if (setTorqueOn)
        Herkulex.torqueON(motors[motorNum].hexID);
    else
        Herkulex.torqueOFF(motors[motorNum].hexID);
}

void ArduinoPoppy::ReadBatteryLevel() {
    float analogValue = analogRead(A10);
    float voltage = 0.0048 * analogValue;
    SERIAL_MONITOR.println(voltage);
}

// Return position in the same range as setPosition
void ArduinoPoppy::ReadIMUData() {
    //  long int t1 = millis();
    //  SERIAL_MONITOR.print(t1);
    //  SERIAL_MONITOR.println("");

    /* Get a new sensor event */
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    /* Display the floating point gyroscope data */
    SERIAL_MONITOR.print(gyroscope.x());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(gyroscope.y());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(gyroscope.z());
    SERIAL_MONITOR.print(",");

    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    /* Display the floating point accelerometr data */
    SERIAL_MONITOR.print(acceleration.x());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(acceleration.y());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(acceleration.z());
    SERIAL_MONITOR.print(",");

    imu::Vector<3> magnometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    /* Display the floating point magnetometer data */
    SERIAL_MONITOR.print(magnometer.x());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(magnometer.y());
    SERIAL_MONITOR.print(",");
    SERIAL_MONITOR.print(magnometer.z());
    SERIAL_MONITOR.println("");
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

    return getFloatFromSerial();
}
