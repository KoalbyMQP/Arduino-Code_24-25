#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <motor_control.h>
#include <Kalman.h>

#include "Kalman.h"

Kalman kalman;  

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
MotorControl stepper = MotorControl(2,5,200);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  // Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);

  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  // Serial.print("Current Temperature: ");
  // Serial.print(temp);
  // Serial.println(" C");
  // Serial.println("");
  stepper.setup();
  stepper.setSpeed(5);
  bno.setExtCrystalUse(true);

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


}




/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{


  //calibration  time :)
 //stepper.spinForRotation(0);
  //delay(10000);
  int delayTime = BNO055_SAMPLERATE_DELAY_MS;
  //int delayTime = 3000;

  while(true){
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    unsigned long currentTime = millis();
    //float dt = (currentTime - prevTime) / 1000.0;  //  time since the calculation was last performe
    float accelX = accel.x();

    //float kalmanX = kalman.kalmanFilter(euler.x());
  // R is the process noise (internal noise of the system)
  // Q is the measurement noise (how much noise is caused by your measurements)
  // A, B, C are the system parameters. The system is described as x[k + 1] = A x[k] + B u[k]



    Kalman *F = new Kalman(0.03, 0.001, 1, 1, 1);
    F->filter(euler.x(), 0);
    

    /* Display the floating point data */
    stepper.spinForRotation(.5);


    /* Display the filtered data */
    Serial.print("IMU_X: ");
    Serial.print(euler.x());
    Serial.print(" FILTERED_X: ");
    Serial.print(F->filter(euler.x(), 0));
    Serial.println("\t\t");

    

    // // Update previous state
    // prevAngle = euler.x();
    // prevTime = currentTime;
    
    delay(delayTime);


    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    currentTime = millis();
    // dt = (currentTime - prevTime) / 1000.0;  // Convert to seconds
    accelX = accel.x(); 
    // kalmanX = kalman.kalmanFilter(euler.x());

    /* Display the floating point data */
    stepper.spinForRotation(-.5);

    /* Display the filtered data */
    Serial.print("IMU_X: ");
    Serial.print(euler.x());
    Serial.print(" FILTERED_X: ");
    Serial.print(F->filter(euler.x(), 0));
    Serial.println("\t\t");

    // Update previous state
    // prevAngle = euler.x();
    // prevTime = currentTime;
    delay(delayTime);




}
}
 