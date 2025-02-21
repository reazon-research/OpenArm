#include "damiao_port.h"
#include <thread>
#include <chrono>

DamiaoPort::DamiaoPort(const std::string& device,
                       const std::vector<DM_Motor_Type>& types, 
                       const std::vector<uint16_t>& can_ids, 
                       const std::vector<uint16_t>& master_ids, 
                       const std::vector<bool>& motor_with_torque, 
                       Control_Type control_mode)
    : canbus_(std::make_unique<CANBus>(device)), 
      control_(*canbus_),
      motor_with_torque_(motor_with_torque) { 

        for (size_t i = 0; i < types.size(); ++i) {
            motors_.push_back(std::make_unique<Motor>(types[i], can_ids[i], master_ids[i]));
        }
    
        for (auto& motor : motors_) {
            control_.addMotor(*motor);  
            control_.enable(*motor);
        }
}

std::vector<std::vector<double>> DamiaoPort::getPresentStatus() {
    std::vector<std::vector<double>> stat;
    for (auto& motor : motors_) {
        stat.push_back({
            motor->getGoalPosition(),
            motor->getGoalTau(),
            motor->getPosition(),
            motor->getVelocity(),
            motor->getTorque(),
            static_cast<double>(motor->getStateTmos()),
            static_cast<double>(motor->getStateTrotor())
        });
    }
    return stat;
}

std::vector<double> DamiaoPort::getPositions() {
    std::vector<double> stat;
    for (auto& motor : motors_) {
        stat.push_back(motor->getPosition());
    }
    return stat;
}

std::vector<double> DamiaoPort::getVelocities() {
    std::vector<double> stat;
    for (auto& motor : motors_) {
        stat.push_back(motor->getVelocity());
    }
    return stat;
}

std::vector<double> DamiaoPort::getTorques() {
    std::vector<double> stat;
    for (auto& motor : motors_) {
        stat.push_back(motor->getTorque());
    }
    return stat;
}

void DamiaoPort::disable() {
    for (auto& motor : motors_) {
        control_.controlMIT(*motor, 0, 0, 0, 0, 0);
        control_.disable(*motor);
    }
}

void DamiaoPort::setZeroPosition() {
    sleep(0.1);
    for (auto& motor : motors_) {
        control_.set_zero_position(*motor);
    }

    sleep(0.1);
}


void DamiaoPort::moveTowardsSync(const std::vector<double>& goal_positions, 
                                 const std::vector<double>& kps, 
                                 const std::vector<double>& kds) {
    for (size_t i = 0; i < motors_.size(); ++i) {
        double delta = goal_positions[i] - motors_[i]->getPosition();
        double v = motors_[i]->getVelocity();
        double tau = kps[i] * delta - kds[i] * v;
        
        motors_[i]->setGoalPosition(goal_positions[i]);
        motors_[i]->setGoalTau(tau);
        
        control_.controlMIT(*motors_[i], 0, 0, 0, 0, tau);
    }
}

void DamiaoPort::setGoalTorqueSync(const std::vector<double>& goal_taus) {
    for (size_t i = 0; i < motors_.size(); ++i) {
        motors_[i]->setGoalPosition(0);
        motors_[i]->setGoalTau(goal_taus[i]);

        control_.controlMIT(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());
        std::this_thread::sleep_for(std::chrono::microseconds(30));
    }
}

void DamiaoPort::moveTorqueSync(const std::vector<double>& taus) {

    for (size_t i = 0; i < motors_.size(); ++i) {
        motors_[i]->setGoalPosition(0);
        motors_[i]->setGoalTau(taus[i]);
        control_.controlMIT(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());
        std::this_thread::sleep_for(std::chrono::microseconds(30));
    }
}

void DamiaoPort::moveTorqueSync2(const std::vector<double>& taus) {

for (size_t i = 0; i < motors_.size(); ++i) {
    motors_[i]->setGoalPosition(0);
    motors_[i]->setGoalTau(taus[i]);
    control_.controlMIT2(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());

}
for (size_t i = 0; i < motors_.size(); ++i) {
    control_.recv();
}

}

void DamiaoPort::keepTorqueSync() {
    for (auto& motor : motors_) {
        control_.controlMIT(*motor, 0, 0, 0, 0, motor->getGoalTau());
        std::this_thread::sleep_for(std::chrono::microseconds(30));
    }
}

void DamiaoPort::setGoalPositionsSync(const std::vector<double>& goal_positions,
                                      const std::vector<double>& kps, 
                                      const std::vector<double>& kds) {
    for (size_t i = 0; i < motors_.size(); ++i) {
        motors_[i]->setGoalPosition(goal_positions[i]);
        motors_[i]->setGoalTau(0);

        control_.controlMIT(*motors_[i], kps[i], kds[i], goal_positions[i], 0, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(30));
    }
}

void DamiaoPort::setMITSync(
    const std::vector<double>& goal_positions,
    const std::vector<double>& goal_velocities,
    const std::vector<double>& kps, 
    const std::vector<double>& kds,
    const std::vector<double>& goal_tau) {
for (size_t i = 0; i < motors_.size(); ++i) {
motors_[i]->setGoalPosition(goal_positions[i]);
motors_[i]->setGoalTau(0);
double kk = 0.75;
control_.controlMIT2(*motors_[i], kps[i]*kk, kds[i]*kk, goal_positions[i]*kk, goal_velocities[i]*kk, goal_tau[i]*kk);
}

std::this_thread::sleep_for(std::chrono::microseconds(10));

for (size_t i = 0; i < motors_.size(); ++i) {
    control_.recv();
}

}