#include <motor_control.h>

/**
 * @brief Class description: This class is designed to facilitate motor control for a stepper
 * motor. It encapsulates functions for setting up the motor, controlling
 * its speed, and executing movements such as spinning for a defined number
 * of steps or rotations. The class handles the necessary timing by calcul-
 * ating and applying appropriate delays between each step. By specifying
 * the appropriate pin assignments, users can easily interface with the mo-
 * tor control system.
 *
 */
/**
 * @brief Construct a new Motor Control:: Motor Control object.
 *
 * @param stepPin stepper motor step pin
 * @param dirPin stepper motor direction pin
 * @param stepsPerRevolution number of steps per revolution for the stepper motor
 */
MotorControl::MotorControl(int stepPin, int dirPin, int stepsPerRevolution)
{
    StepX = stepPin;
    DirX = dirPin;
    this->stepsPerRevolution = stepsPerRevolution;
}
/**
 * @brief Setup method to apply pin values to stepper motor.
 *
 */
void MotorControl::setup()
{
    pinMode(StepX, OUTPUT);
    pinMode(DirX, OUTPUT);
}
// Setters----------------------------------------
/**
 * @brief Set the motor bounds for the stepper motor.
 *
 * @param minTargetPosition
 * @param maxTargetPosition
 */
void MotorControl::setMotorBounds(int minTargetPosition, int maxTargetPosition)
{

    this->minTargetPosition = minTargetPosition;
    this->maxTargetPosition = maxTargetPosition;
    Serial.println("Min: " + String(minTargetPosition));
    Serial.println("Max: " + String(maxTargetPosition));
}
/**
 * @brief Set the unit of measure for the stepper motor.
 *
 * @param uom IN or CM
 */
void MotorControl::setUnitOfMeasure(UnitOfMeasure uom)
{
    this->uom = uom;
}
/**
 * @brief Set the amount of linear distance per revolution of the stepper motor.
 *
 * @param inchesPerRevolution
 */
void MotorControl::setLinearDistancePerRevolution(float inchesPerRevolution)
{
    this->inchesPerRevolution = inchesPerRevolution;
}
/**
 * @brief Set the Speed object (revolutions per second) and do math to find delay per step .
 *
 * @param speed revolutions per second
 */
void MotorControl::setSpeed(float speed)
{
    this->speed = speed;
    delayPerStep = 1000000.0 / (stepsPerRevolution * speed);
}

/**
 * @brief Spin the stepper motor to a given target position at a previously defined speed.
 *
 * @param targetPosition target position to spin to in steps
 */
void MotorControl::setTargetPosition(int targetPositionSTEPS)
{
    this->targetPosition = targetPositionSTEPS;
    // int steps = targetPosition - currentPosition;
    // spinForSteps(steps);
}
/**
 * @brief Spin the stepper motor until a given linear target position is reached.
 *
 * @param targetPositionIN Inches (relative)
 */
void MotorControl::setLinearTargetPosition(float targetPositionIN)
{
    this->targetPosition = constrain((targetPositionIN * stepsPerRevolution) / inchesPerRevolution, minTargetPosition, maxTargetPosition);
}
/**
 * @brief Spin the stepper motor for a given number of steps at a previously defined speed.
 *
 * @param steps number of steps to spin
 */
void MotorControl::spinForSteps(int steps)
{
    if (steps > 0)
        digitalWrite(DirX, LOW);
    else
        digitalWrite(DirX, HIGH);

    for (int x = 0; x < abs(steps); x++)
    {
        digitalWrite(StepX, HIGH);
        delayMicroseconds(static_cast<int>(delayPerStep));
        digitalWrite(StepX, LOW);
        delayMicroseconds(static_cast<int>(delayPerStep));
        if (steps > 0)
            currentPosition++;
        else
            currentPosition--;
    }

    // Serial.println("Current Position: " + String(currentPosition));
}

/**
 * @brief Spin the stepper motor to a given target position at
 *  a speed determined by its distance from its target position.
 *
 */
void MotorControl::spinForPID()
{
    // PID controller parameters
    float kp = .02;
    float ki = 0;  // Set to 0 for proportional drive
    float kd = .7; // Set to 0 for proportional drive

    // Initialize variables
    float integral = 0.0;
    float previousError = 0.0;

    // Control loop
    while (currentPosition != targetPosition)
    {
        // Calculate the error
        int error = abs(targetPosition - currentPosition);
        if (targetPosition - currentPosition < 0)
            digitalWrite(DirX, HIGH);
        else
            digitalWrite(DirX, LOW);
        // Calculate the proportional term
        float proportional = kp * error;

        // Calculate the integral term (optional)
        integral += ki * error;

        // Calculate the derivative term (optional)
        float derivative = kd * (error - previousError);
        previousError = error;

        // Combine the terms
        float controlSignal = constrain(proportional + integral + derivative, 2, 12);
        // Adjust the motor speed based on the control signal
        setSpeed(abs(controlSignal));

        // Perform the step
        digitalWrite(StepX, HIGH);
        delayMicroseconds(static_cast<int>(delayPerStep));
        digitalWrite(StepX, LOW);
        delayMicroseconds(static_cast<int>(delayPerStep));

        // Update the current position
        if (targetPosition - currentPosition > 0)
            currentPosition++;
        else
            currentPosition--;

        // Update the integral term (optional)
        integral = constrain(integral, -1000.0, 1000.0);
    }
}

/**
 * @brief Spin the stepper motor for a given number of rotations at a previously defined speed.
 *
 * @param rotations number of rotations to spin
 */
void MotorControl::spinForRotation(float rotations)
{
    float steps = rotations * stepsPerRevolution;
    spinForSteps((int)steps);
}
