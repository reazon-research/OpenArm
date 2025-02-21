#include <vector>
#include "dynamics.h"

Dynamics::Dynamics(std::string urdf_path, std::string start_link, std::string end_link)
{
  this->urdf_path = urdf_path;
  this->start_link = start_link;
  this->end_link = end_link;
}

Dynamics::~Dynamics(){}

bool Dynamics::Init()
{
  if (!urdf_model.initFile(urdf_path)) {
    fprintf(stderr, "Failed to parse URDF: %s\n", urdf_path.c_str());
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    fprintf(stderr, "Failed to extract KDL tree: %s\n", urdf_path.c_str());
    return false;
  }

  if (!kdl_tree.getChain(start_link, end_link, kdl_chain)) {
    fprintf(stderr, "Failed to not get KDL chain");
    return false;
  }

  coriolis_forces.data.setZero();
  gravity_forces.data.setZero();
  inertia_matrix.data.setZero();

  coriolis_forces.resize(kdl_chain.getNrOfJoints());
  gravity_forces.resize(kdl_chain.getNrOfJoints());
  inertia_matrix.resize(kdl_chain.getNrOfJoints());

  // solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0.0, 0.0, -9.81));

  // 9.81 is the gravitational constant (inverted)
  // because OpenArm is mounted upside down
  solver = std::make_unique<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0.0, 9.81, 0.0));

  return true;
}

void Dynamics::GetGravity(const double *motor_position, double *gravity)
{
  KDL::JntArray q_(kdl_chain.getNrOfJoints());
  
  for(uint8_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    q_(i) = motor_position[i];

    // if (i == 1) {
    //   q_(i) -= 1.4;
    // }
    
    // if (i == 3) {
    //   q_(i) -= 2.7;
    // }

    if (i == 1) {
      q_(i) -= -0.2;
    }
    
    if (i == 3) {
      q_(i) -= 2.7;
    }


  }
  solver->JntToGravity(q_, gravity_forces);
  for(uint8_t i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    gravity[i] = gravity_forces(i);
  }
}

void Dynamics::GetColiori(const double *motor_position, const double *motor_velocity, double *colioli) {
  KDL::JntArray q_(kdl_chain.getNrOfJoints());
  KDL::JntArray q_dot(kdl_chain.getNrOfJoints());
  uint8_t i;

  for(i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    q_(i) = motor_position[i];
    q_dot(i) = motor_velocity[i];
  }

  solver->JntToCoriolis(q_, q_dot, coriolis_forces);

  for(i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    colioli[i] = coriolis_forces(i);
  }
}

void Dynamics::GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag)
{
  KDL::JntArray q_(kdl_chain.getNrOfJoints());
  KDL::JntSpaceInertiaMatrix inertia_matrix(kdl_chain.getNrOfJoints());
  uint8_t i;

  for(i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    q_(i) = motor_position[i];
  }

  solver->JntToMass(q_, inertia_matrix);

  for(i = 0; i < kdl_chain.getNrOfJoints(); i++) {
    inertia_diag[i] = inertia_matrix(i, i);
  }
}