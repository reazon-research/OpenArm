#ifndef REAZON_BILATERAL_DYNAMICS_H
#define REAZON_BILATERAL_DYNAMICS_H

#include <unistd.h>
#include <string.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "global.h"

/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics
{
 private:
  urdf::Model urdf_model;

  std::string urdf_path;
  std::string start_link;
  std::string end_link;

  KDL::JntSpaceInertiaMatrix inertia_matrix;
  KDL::JntArray q;
  KDL::JntArray q_d;
  KDL::JntArray coriolis_forces;
  KDL::JntArray gravity_forces;

  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;
  std::unique_ptr<KDL::ChainDynParam> solver;

 public:
  Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
  ~Dynamics();
  bool Init();
  void GetGravity(const double *motor_position, double *gravity);
  void GetColiori(const double *motor_position, const double *motor_velocity, double *colioli);
  void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);
};

#endif
