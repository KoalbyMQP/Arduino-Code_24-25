
#include <Arduino.h>
#include <Stepper.h>
#include <stdlib.h>
#include <string.h>

class MotorControl
{

public:
    enum UnitOfMeasure
    {
        IN,
        CM
    };
    MotorControl(int stepPin, int dirPin, int stepsPerRevolution);
    // Setters----------------------------------------
    void setUnitOfMeasure(UnitOfMeasure uom);
    void setMotorBounds(int minTargetPosition, int maxTargetPosition);
    void setLinearTargetPosition(float targetPositionIN);
    void setLinearDistancePerRevolution(float inchesPerRevolution);
    void setSpeed(float speed = 10);
    void setTargetPosition(int targetPostion);
    // functional methods----------------------------------------
    void spinForPID();
    void spinForSteps(int steps);
    void spinForRotation(float rotations);
    void runSpeed(float speed);
    void setup();

protected:
    UnitOfMeasure uom;
    float inchesPerRevolution;
    int maxTargetPosition = 0;
    int minTargetPosition = 0;

    int StepX;
    int DirX;
    int stepsPerRevolution;
    int currentPosition = 0; // Variable to track the current position
    float speed = 0;         // Variable to set the current speed in revs per second
    float delayPerStep = 0;
    float acceleration = 0;
    int targetPosition = 0;
};