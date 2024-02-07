#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <motor_control.h>


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


const float alpha = 0.61; // Adjust this value based on the filtering effect you desire

// Variables to store filtered IMU values
float filteredX = 0.0;


//Function to apply a filter to the IMU values
void applyLowPassFilter(imu::Vector<3>& input, float& output) {
  output = alpha * input.x() + (1.0 - alpha) * output;
  if(output > 360){
    output = 360 - output;
  }
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
  // stepper.spinForRotation(0);
  // delay(5000);
  int delayTime = BNO055_SAMPLERATE_DELAY_MS;
  //int delayTime = 0;

  while(true){

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  
  /* Display the floating point data */
  applyLowPassFilter(euler, filteredX);
  stepper.spinForRotation(.5);
  

  //delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(delayTime);


  Serial.print("IMU_X: ");
  Serial.print(euler.x());
  Serial.print(" FIXED_X: ");
  Serial.print(filteredX);
 
  Serial.println();

  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  applyLowPassFilter(euler, filteredX);
  stepper.spinForRotation(-.5);



  Serial.print("IMU_X: ");
  Serial.print(euler.x());
  Serial.print(" FIXED_X: ");
  Serial.print(filteredX);

  Serial.println();


  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  //delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(delayTime);
  

}
}