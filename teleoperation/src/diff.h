#ifndef REAZON_BILATERAL_ALOHA_DIFF_H_
#define REAZON_BILATERAL_ALOHA_DIFF_H_

#include "global.h"

class Differentiator
{
private:
  double Ts_;                            // Sampling time
  double velocity_z1_[NMOTORS] = {0.0};  // Velocity (1 step before)
  double position_z1_[NMOTORS] = {0.0};  // Position (1 step before)

public:
  Differentiator(double Ts) : Ts_(Ts) {}

  /*
   * Compute the motor speed by taking the derivative of
   * the motion.
   */
  void Differentiate(const double *position, double *velocity)
  {
    double a = 1.0 / (1.0 + Ts_ * CUTOFF_FREQUENCY);
    double b = a * CUTOFF_FREQUENCY;

    for (int i = 0; i < NMOTORS; i++) {
      if (position_z1_[i] == 0.0) {
        position_z1_[i] = position[i];
      }

      // Because sensor signals contain a lot of noises, we employ
      // a low-pass filter for smoothing purposes.
      velocity[i] = velocity_z1_[i] * a + b * (position[i] - position_z1_[i]);
      position_z1_[i] = position[i];
      velocity_z1_[i] = velocity[i];
    }
  }
};

#endif
