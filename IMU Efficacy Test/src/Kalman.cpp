// #include "Kalman.h"
// #include <cmath>

// #define ANGLE_FILTER_KAPPA = 
// #define pitch
// #define pitch_bias
// #define DELTA_TIME_S
// #define PI


// double kalmanFilter(double z){
//   static const double R = 0.001; // measurement noise covariance
//   static const double HT = 1.00; //measurement function
//   //static double Q = 10; // motion noise
//   static double P = 1.0; // uncertainty Covariance
//   static double X_hat = 0.0; // initial estimated state 
//   static double K = 0.0; // Kalman Gain
//   static double I = 1.0; //identity matrix

//   ///////// (P × HT)/((H×P×HT)+R)          
//   K = (P*HT)/((HT*P*HT)+R); //Kalman Gain
//   X_hat = X_hat + K*(z-HT*X_hat); //update the estimate with measurement

//   P = (I - K*HT) * P; // updating uncertainty covariance

//   return X_hat;
// }


// void updatePitch()
// {
//     double gyro_pitch_dps = (imu.g.y - pitch_bias) * imu.mdpsPerLSB / 1000.0;
//     double prediction = pitch + (DELTA_TIME_S * gyro_pitch_dps * PI / 180);
//     double imuX = imu.a.x * imu.mgPerLSB;
//     double imuZ = imu.a.z * imu.mgPerLSB;
//     double observation = atan2(imuX, imuZ);
//     pitch = prediction + ANGLE_FILTER_KAPPA * (observation - prediction);
//     pitch_bias = pitch_bias - GYRO_BIAS_EPSILON * (GYRO_NOISE_STDEV / DELTA_TIME_S) * (observation - prediction);
// }


#include "Kalman.h"

Kalman::Kalman(float r, float q, float a, float b, float c) {
  R = r;
  Q = q;
  A = a;
  B = b;
  C = c;
  init = false;
}

float Kalman::filter(float z, float u) {
  if (!init) {
    x = 1 / C * z;
    cov = Q / (C * C);
    init = true;
  }

  else {
    float pred = predict(u);
    float p_cov = uncertainty();

    float K = p_cov * C / (C * C * p_cov + Q);

    x = pred + K * (z - C * pred);
    cov = p_cov - K * C * p_cov;

  }

  return x;

}

float Kalman::predict(float u) {
  return A * x + B * u;
}

float Kalman::uncertainty() {
  return A * A * cov + R;
}