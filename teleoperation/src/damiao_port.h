#ifndef DAMIAO_PORT_H
#define DAMIAO_PORT_H

#include "motor_control.h"
#include "motor.h"
#include "canbus.h"
#include <vector>
#include <memory>  


class DamiaoPort {
public:
    DamiaoPort(const std::string& device, 
               const std::vector<DM_Motor_Type>& types, 
               const std::vector<uint16_t>& can_ids, 
               const std::vector<uint16_t>& master_ids, 
               const std::vector<bool>& motor_with_torque, 
               Control_Type control_mode = Control_Type::MIT);

    std::vector<std::vector<double>> getPresentStatus();
    std::vector<double> getPositions();
    std::vector<double> getVelocities();
    std::vector<double> getTorques();

    void disable();
    void setZeroPosition();
    void moveTowardsSync(const std::vector<double>& goal_positions, 
                         const std::vector<double>& kps, 
                         const std::vector<double>& kds);
    void setGoalTorqueSync(const std::vector<double>& goal_taus);
    void moveTorqueSync(const std::vector<double>& taus);
    void moveTorqueSync2(const std::vector<double>& taus);
    void keepTorqueSync();
    void setGoalPositionsSync(const std::vector<double>& goal_positions, 
                              const std::vector<double>& kps, 
                              const std::vector<double>& kds);

    void setMITSync(
    const std::vector<double>& goal_positions,
    const std::vector<double>& goal_velocities,
    const std::vector<double>& kps, 
    const std::vector<double>& kds,
    const std::vector<double>& goal_tau);
    
    // const std::vector<Motor>& getMotors() const { return motors_; }
    const std::vector<std::unique_ptr<Motor>>& getMotors() const { return motors_; }

private:
    std::unique_ptr<CANBus> canbus_; 
    MotorControl control_;
    std::vector<std::unique_ptr<Motor>> motors_; 
    std::vector<bool> motor_with_torque_; 
};

#endif // DAMIAO_PORT_H