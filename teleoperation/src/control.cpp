#include <cmath>
#include <unistd.h>
#include <string.h>
#include <thread>
#include "control.h"
#include <cmath>


Control::Control(DamiaoPort *arm, double Ts, int role):
  arm_(arm), Ts_(Ts), role_(role)
{
  differentiator_ = new Differentiator(Ts);
  response_ = new sensor_msgs::msg::JointState();
  reference_ = new sensor_msgs::msg::JointState();
}

bool Control::Setup(void)
{
  double motor_position[NMOTORS] = {0.0};

  response_->position.resize(NJOINTS, 0.0);
  response_->velocity.resize(NJOINTS, 0.0);
  response_->effort.resize(NJOINTS, 0.0);

  reference_->position.resize(NJOINTS, 0.0);
  reference_->velocity.resize(NJOINTS, 0.0);
  reference_->effort.resize(NJOINTS, 0.0);

  dynamics_l_ = new Dynamics(LEADER_URDF_PATH, LEADER_START_LINK, LEADER_END_LINK);
  dynamics_f_ = new Dynamics(FOLLOWER_URDF_PATH, FOLLOWER_START_LINK, FOLLOWER_END_LINK);

  if (!dynamics_l_->Init()) {
    return false;
  }
  if (!dynamics_f_->Init()) {
    return false;
  }

  ComputeJointPosition(motor_position, response_->position.data());

  // Variables for Debug()
  ncycle = 0;
  t0 = calcTime();

  std::cout << "!control_f->Setup()  finished "<< std::endl;

  return true;
}

void Control::Shutdown(void)
{
  arm_->disable();
}

void Control::Configure(const double *Dn, const double *Gn, const double *Jn, const double *gn,
                        const double *Kp, const double *Kd, const double *Kf)
{
  memcpy(Dn_, Dn, sizeof(double) * NJOINTS);
  memcpy(Gn_, Gn, sizeof(double) * NJOINTS);
  memcpy(Jn_, Jn, sizeof(double) * NJOINTS);
  memcpy(gn_, gn, sizeof(double) * NJOINTS);
  memcpy(Kp_, Kp, sizeof(double) * NJOINTS);
  memcpy(Kd_, Kd, sizeof(double) * NJOINTS);
  memcpy(Kf_, Kf, sizeof(double) * NJOINTS);
}

// Multithreading and KDL-enabled bilateral control
bool Control::DoControl()
{
  double motor_position[NMOTORS] = {0.0};
  double motor_velocity[NMOTORS] = {0.0};
  double motor_torque[NMOTORS] = {0.0};
  double joint_torque[NJOINTS] = {0.0};
  double friction[NJOINTS] = {0.0};
  double gravity[NJOINTS] = {0.0};
  double colioli[NJOINTS] = {0.0};
  double inertia_diag[NJOINTS] = {0.0};

  //leader and followet inertia
  double inertia_diag_l[NJOINTS] = {0.0};
  double inertia_diag_f[NJOINTS] = {0.0};

  // std::vector<double> kp_temp = {120, 110.0, 110.0, 110.0, 10.0, 7.0, 10.0, 0.85};
  // std::vector<double> kd_temp = {2.3, 1.5, 1.5, 1.5, 0.3, 0.2, 0.3, 0.04};

  std::vector<double> kp_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> kd_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::vector<double> positions = arm_->getPositions();
  std::vector<double> velocities = arm_->getVelocities();

  // if (role_ == ROLE_LEADER){
  //   // for(int i = 0; i < NMOTORS; i++)
  //   // {
  //   //   std::cout << "leader position : " << positions[i] << std::endl;

  //   // }

  //   std::cout << "position leader 8 : " << positions[7] << std::endl;
  //   std::cout << "================================================" << std::endl;

  // }
  // if (role_ == ROLE_FOLLOWER){
    // for(int i = 0; i < NMOTORS; i++)
    // {
    //   std::cout << " follwer position : " << positions[i] << std::endl;

    // }
  // std::cout << "position leader 8 : " << positions[7] << std::endl;
  // std::cout << "================================================" << std::endl;

  // }


  differentiator_->Differentiate(motor_position, motor_velocity);

  for (size_t i = 0; i < positions.size(); ++i) {
    motor_position[i] = positions[i];
    motor_velocity[i] = velocities[i];
  }



  ncycle++;

  // Get joint states based on actuator states
  ComputeJointPosition(motor_position, response_->position.data());
  ComputeJointVelocity(motor_velocity, response_->velocity.data());

  // Friction (speed x constant coeff)
  ComputeFriction(response_->velocity.data(), friction);

  // Compute inertia based on oblique coordinate system
  if (role_ == ROLE_LEADER) {
    dynamics_l_->GetGravity(response_->position.data(), gravity);
    dynamics_l_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
    dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

    dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag_l);
    dynamics_f_->GetMassMatrixDiagonal(reference_->position.data(), inertia_diag_f);
    inertia_diag_l[NJOINTS - 1] = 0.01;
    inertia_diag_f[NJOINTS - 1] = 0.01;
  } else {
    dynamics_f_->GetGravity(response_->position.data(), gravity);
    dynamics_f_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
    dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

    dynamics_l_->GetMassMatrixDiagonal(reference_->position.data(), inertia_diag_l);
    dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag_f);
    inertia_diag_l[NJOINTS - 1] = 0.01;
    inertia_diag_f[NJOINTS - 1] = 0.01;
  }

  for (int i = 0; i < NJOINTS; i++) {
    if (i >= 1 && i < 4)
        Jn_[i] = inertia_diag[i];

    if (role_ == ROLE_LEADER) {
      oblique_coordinates_force = (ALPHA * inertia_diag_l[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
      oblique_coordinates_position = (BETA * inertia_diag_l[i] * inertia_diag_f[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
    } else {
      oblique_coordinates_force = (inertia_diag_f[i]) / (ALPHA  * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
      oblique_coordinates_position = (inertia_diag_l[i] * inertia_diag_f[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
    }

    // double tau_p_oblique = oblique_coordinates_position * Kp_[i] * (reference_->position[i] - response_->position[i]);
    // double tau_v_oblique = oblique_coordinates_position * Kd_[i] * (reference_->velocity[i] - response_->velocity[i]);
    // double tau_f_oblique =  - oblique_coordinates_force * Kf_[i] * (reference_->effort[i] + response_->effort[i]);


    double tau_p_oblique = kp_temp[i] * (reference_->position[i] - response_->position[i]);
    double tau_v_oblique = kd_temp[i] * (reference_->velocity[i] - response_->velocity[i]);
    double tau_f_oblique = - oblique_coordinates_force * Kf_[i] * (reference_->effort[i] + response_->effort[i]);

    // joint_torque[i] = tau_p_oblique + tau_v_oblique + gravity[i] * 0.9 + tau_f_oblique + disturbance_[i];

    // if( i >= 4 && i <= 7){
    //   joint_torque[i] = tau_p_oblique + tau_v_oblique + gravity[i] * 0.9 + friction[i];
    // }

    // if (i == 8){
    //   joint_torque[i] = tau_p_oblique + tau_v_oblique + tau_f_oblique + disturbance_[i];
    // }

    // for debugging 
    joint_torque[i] = gravity[i] * 0.1;


    // DOB 1 Mass jointspace
    double a = gn_[i] * Ts_;
    disturbance_lowpassin_[i] = (joint_torque[i] - gravity[i]) + gn_[i] * Jn_[i] * response_->velocity[i];
    disturbance_lowpassout_[i] += a * (disturbance_lowpassin_[i] - disturbance_lowpassout_[i]);
    disturbance_[i] = disturbance_lowpassout_[i] - response_->velocity[i] * Jn_[i] * gn_[i];

    // RFOB 1 Mass jointspace
    double a_ = gn_[i] * Ts_;
    reactionforce_lowpassin_[i] = (joint_torque[i] - gravity[i]) + gn_[i] * Jn_[i] * response_->velocity[i] - friction[i] -  colioli[i];
    reactionforce_lowpassout_[i] += a_ * (reactionforce_lowpassin_[i] - reactionforce_lowpassout_[i]);
    response_->effort[i] = reactionforce_lowpassout_[i] - response_->velocity[i] * Jn_[i] * gn_[i];

    // temp added
    joint_torque[i] += (-tau_p_oblique - tau_v_oblique);

  }

  // debug
  // std::vector<double> kp_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // std::vector<double> kd_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  ComputeMotorTorque(joint_torque, motor_torque);

  // return arm_->SetTorque(motor_torque);
  std::vector<double> damiao_torque;

  for (size_t i = 0; i < positions.size(); ++i) {
    damiao_torque.push_back(motor_torque[i]);
  }

  // arm_->moveTorqueSync2(damiao_torque);
  arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);

  return true;
}


bool Control::DoControl_u(){

  double motor_position[NMOTORS] = {0.0};
  double motor_velocity[NMOTORS] = {0.0};
  double motor_torque[NMOTORS] = {0.0};
  double joint_torque[NJOINTS] = {0.0};
  double friction[NJOINTS] = {0.0};
  double gravity[NJOINTS] = {0.0};
  double colioli[NJOINTS] = {0.0};
  double inertia_diag[NJOINTS] = {0.0};

  std::vector<double> positions = arm_->getPositions();
  std::vector<double> velocities = arm_->getVelocities();

  for (size_t i = 0; i < positions.size(); ++i) {
    motor_position[i] = positions[i];
    motor_velocity[i] = velocities[i];
  }

  ComputeJointPosition(motor_position, response_->position.data());
  ComputeJointVelocity(motor_velocity, response_->velocity.data());

  if(role_ == ROLE_LEADER){
    
    ComputeFriction(response_->velocity.data(), friction);

    dynamics_l_->GetGravity(response_->position.data(), gravity);
    dynamics_l_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
    dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

    for (int i = 0; i < NJOINTS; i++) {
      joint_torque[i] = gravity[i]*0.7 + friction[i]*0.08 + colioli[i];
    }

    ComputeMotorTorque(joint_torque, motor_torque);
    // return arm_->SetTorque(motor_torque);
    std::vector<double> damiao_torque;

    for (size_t i = 0; i < positions.size(); ++i) {
      damiao_torque.push_back(motor_torque[i]);
    }

    std::vector<double> kp_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> kd_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};

    // arm_->moveTorqueSync2(damiao_torque);
    arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);

    return true;

  }

  else if(role_ == ROLE_FOLLOWER){

    dynamics_f_->GetGravity(response_->position.data(), gravity);
    dynamics_f_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
    dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

    std::vector<double> damiao_torque;

    for (size_t i = 0; i < positions.size(); ++i) {
      damiao_torque.push_back(motor_torque[i]);
    }

    std::vector<double> kp_temp = {120, 110.0, 110.0, 110.0, 10.0, 7.0, 10.0, 0.85};
    std::vector<double> kd_temp = {2.3, 1.5, 1.5, 1.5, 0.3, 0.2, 0.3, 0.04};

    arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);

    return true;
  }

  return true;

}



void Control::ComputeJointPosition(const double *motor_position, double *joint_position)
{

  joint_position[0] = motor_position[0];
  joint_position[1] = motor_position[1];
  joint_position[2] = motor_position[2];
  joint_position[3] = motor_position[3];
  joint_position[4] = motor_position[4];
  joint_position[5] = motor_position[5];
  joint_position[6] = motor_position[6];
  joint_position[7] = motor_position[7];
  
}

void Control::ComputeJointVelocity(const double *motor_velocity, double *joint_velocity)
{

  joint_velocity[0] = motor_velocity[0];
  joint_velocity[1] = motor_velocity[1];
  joint_velocity[2] = motor_velocity[2];
  joint_velocity[3] = motor_velocity[3];
  joint_velocity[4] = motor_velocity[4];
  joint_velocity[5] = motor_velocity[5];
  joint_velocity[6] = motor_velocity[6];
  joint_velocity[7] = motor_velocity[7];

}

void Control::ComputeMotorTorque(const double *joint_torque, double *motor_torque)
{
  motor_torque[0] = joint_torque[0];
  motor_torque[1] = joint_torque[1];
  motor_torque[2] = joint_torque[2];
  motor_torque[3] = joint_torque[3];
  motor_torque[4] = joint_torque[4];
  motor_torque[5] = joint_torque[5];
  motor_torque[6] = joint_torque[6];
  motor_torque[7] = joint_torque[7];

}

void Control::ComputeFriction(const double *velocity, double *friction)
{
  for (int i = 0; i < NJOINTS; i++) {
    friction[i] = velocity[i] * Dn_[i];
  }
}

void Control::GetResponse(sensor_msgs::msg::JointState *response)
{
  response->position.resize(NJOINTS, 0.0);
  response->velocity.resize(NJOINTS, 0.0);
  response->effort.resize(NJOINTS, 0.0);

  for (int i = 0; i < NJOINTS; i++) {
    response->position[i] = response_->position[i];
    response->velocity[i] = response_->velocity[i];
    response->effort[i] = response_->effort[i];
  }
}

void Control::SetReference(sensor_msgs::msg::JointState *reference)
{
  reference->position.resize(NJOINTS, 0.0);
  reference->velocity.resize(NJOINTS, 0.0);
  reference->effort.resize(NJOINTS, 0.0);

  for (int i = 0; i < NJOINTS; i++) {
    reference_->position[i] = reference->position[i];
    reference_->velocity[i] = reference->velocity[i];
    reference_->effort[i] = reference->effort[i];
  }
}

bool Control::AdjustPosition(void)
{
  double motor_position[NMOTORS] = {0.0};
  int nstep = 220;
  double a;

  std::vector<double> goalpos(NMOTORS, 0.0);

  std::vector<double> positions_now = arm_->getPositions();
  std::vector<double> velocities_now = arm_->getVelocities();

  ComputeJointPosition(positions_now.data(), response_->position.data());
  ComputeJointVelocity(velocities_now.data(), response_->velocity.data());

  std::vector<double> kp_temp = {120, 260.0, 110.0, 260.0, 10.0, 60.0, 10.0, 1.0};
  std::vector<double> kd_temp = {2.3, 1.5, 1.5, 1.5, 0.3, 0.2, 0.3, 0.01};

  for (int step = 0; step < nstep; step++) {
    a = static_cast<double>(step + 1) / nstep;

    for (int i = 0; i < NMOTORS; i++) {
      goalpos[i] = INITIAL_POSITION[i] * a + positions_now[i] * (1.0 - a);
    }

    arm_->setGoalPositionsSync(goalpos, kp_temp, kd_temp);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ComputeJointPosition(goalpos.data(), response_->position.data());

  return true;
}


void Control::Debug(void)
{
  double elapsed = calcTime() - t0;
  printf("% 5.1lf|%.1lf|", elapsed, ncycle / elapsed);
  for (int i = 0; i < NJOINTS; i++) {
    printf("%i %+3.1lf %+3.1lf %+3.1lf|", i, response_->position[i], response_->velocity[i], response_->effort[i]);
  }
  for (int i = 0; i < NJOINTS; i++) {
    printf("%i %+3.1lf %+3.1lf %+3.1lf|", i, reference_->position[i], reference_->velocity[i], reference_->effort[i]);
  }
  printf("\n");
}
