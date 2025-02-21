#ifndef REAZON_BILATERAL_OPENARM_CONTROL_H_
#define REAZON_BILATERAL_OPENARM_CONTROL_H_

#include <sensor_msgs/msg/joint_state.hpp>
#include "global.h"
#include "damiao_port.h"
#include "diff.h"
#include "dynamics.h"

/*
 * Implement Bilateral control on top of the low-level
 * manipulator class (Check out arm.h).
 */

class Control
{
  // BaseArm *arm_;
  DamiaoPort *arm_;
  double Ts_;
  int role_;
  Differentiator *differentiator_;

  // Debug variables
  int ncycle;
  int t0;

  Dynamics *dynamics_f_;
  Dynamics *dynamics_l_;

  float oblique_coordinates_force;
  float oblique_coordinates_position;

  // DOB-estimated disturbance torque [Nm]
  double disturbance_lowpassin_[NJOINTS] = {0.0};
  double disturbance_lowpassout_[NJOINTS] = {0.0};
  double disturbance_[NJOINTS] = {0.0};

  double reactionforce_lowpassin_[NJOINTS] = {0.0};
  double reactionforce_lowpassout_[NJOINTS] = {0.0};

 public:
  sensor_msgs::msg::JointState *response_;
  sensor_msgs::msg::JointState *reference_;

  double Dn_[NJOINTS] = {0.0};
  double Gn_[NJOINTS] = {0.0};
  double Jn_[NJOINTS] = {0.0};
  double gn_[NJOINTS] = {0.0};
  double Kp_[NJOINTS] = {0.0};
  double Kd_[NJOINTS] = {0.0};
  double Kf_[NJOINTS] = {0.0};

  Control(DamiaoPort *arm, double Ts, int role);
  bool Setup(void);
  void Setstate(int state);
  void Shutdown(void);

  void Configure(const double *Dn, const double *Gn, const double *Jn, const double *gn,
                 const double *Kn, const double *Kd, const double *Kf);

  // Return a copy of response (or reference)
  void GetResponse(sensor_msgs::msg::JointState *response);
  void SetReference(sensor_msgs::msg::JointState *reference);

  bool AdjustPosition(void);

  // Compute torque based on bilateral
  bool DoControl();
  bool DoControl_u();

  // NOTE! Control() class operates on "joints", while the underlying
  // classes operates on "actuators". The following functions map
  // joints to actuators.
  void ComputeJointPosition(const double *motor_position, double *joint_position);
  void ComputeJointVelocity(const double *motor_velocity, double *joint_velocity);
  void ComputeMotorTorque(const double *joint_torque, double *motor_torque);
  void ComputeFriction(const double *velocity, double *friction);
  void ComputeGravity(const double *position, double *gravity);

  void Debug(void);
};
#endif
