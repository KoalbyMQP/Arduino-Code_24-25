// class Kalman {
// public:
//     Kalman();

//     double kalmanFilter(double z);
//     void updatePitch();


// };

class Kalman {
public:
  float R, Q, A, B, C; 
  float cov;
  float x; // Signal without noise
  bool init;

// R is the process noise (internal noise of the system)
// Q is the measurement noise (how much noise is caused by your measurements)
// A, B, C are the system parameters. The system is described as x[k + 1] = A x[k] + B u[k]

  Kalman(float r, float q, float a, float b, float c);

  float filter(float z, float u);

  float predict(float u);

  float uncertainty();

};