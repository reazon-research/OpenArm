#include "motor_control.h"
#include "motor.h"

MotorControl::MotorControl(CANBus& canbus) : canbus_(canbus) {}

bool MotorControl::addMotor(Motor& motor) {
    motors_map[motor.SlaveID] = &motor;  
    if (motor.MasterID != 0) {
        motors_map[motor.MasterID] = &motor; 
    }
    return true; 
}

void MotorControl::enable(Motor& motor) {
    controlCmd(motor, 0xFC);  
    sleep(0.3);
}

void MotorControl::disable(Motor& motor) {
    controlCmd(motor, 0xFD);  
    sleep(0.3);
}

void MotorControl::set_zero_position(Motor& motor){
    controlCmd(motor, 0xFE);
    sleep(0.3);
    recv();
}

void MotorControl::controlMIT(Motor& motor, double kp, double kd, double q, double dq, double tau) {
    if (motors_map.find(motor.SlaveID) == motors_map.end()) {
        std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
        return;
    }

    uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

    int motor_index = static_cast<int>(motor.MotorType);
    double Q_MAX = Limit_Param[motor_index][0];
    double DQ_MAX = Limit_Param[motor_index][1];
    double TAU_MAX = Limit_Param[motor_index][2];

    uint16_t q_uint = double_to_uint(q, -Q_MAX, Q_MAX, 16);
    uint16_t dq_uint = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
    uint16_t tau_uint = double_to_uint(tau, -TAU_MAX, TAU_MAX, 12);

    std::array<uint8_t, 8> data = {
        static_cast<uint8_t>((q_uint >> 8) & 0xFF),
        static_cast<uint8_t>(q_uint & 0xFF),
        static_cast<uint8_t>(dq_uint >> 4),
        static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
        static_cast<uint8_t>(kp_uint & 0xFF),
        static_cast<uint8_t>(kd_uint >> 4),
        static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
        static_cast<uint8_t>(tau_uint & 0xFF)
    };

    sendData(motor.SlaveID, data);
    recv();
}

void MotorControl::controlMIT2(Motor& motor, double kp, double kd, double q, double dq, double tau) {
    
    if (motors_map.find(motor.SlaveID) == motors_map.end()) {
        std::cerr << "controlMIT ERROR: Motor ID not found" << std::endl;
        return;
    }

    uint16_t kp_uint = double_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = double_to_uint(kd, 0, 5, 12);

    int motor_index = static_cast<int>(motor.MotorType);
    double Q_MAX = Limit_Param[motor_index][0];
    double DQ_MAX = Limit_Param[motor_index][1];
    double TAU_MAX = Limit_Param[motor_index][2];

    uint16_t q_uint = double_to_uint(q, -Q_MAX, Q_MAX, 16);
    uint16_t dq_uint = double_to_uint(dq, -DQ_MAX, DQ_MAX, 12);
    uint16_t tau_uint = double_to_uint(tau, -TAU_MAX, TAU_MAX, 12);

    std::array<uint8_t, 8> data = {
        static_cast<uint8_t>((q_uint >> 8) & 0xFF),
        static_cast<uint8_t>(q_uint & 0xFF),
        static_cast<uint8_t>(dq_uint >> 4),
        static_cast<uint8_t>(((dq_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)),
        static_cast<uint8_t>(kp_uint & 0xFF),
        static_cast<uint8_t>(kd_uint >> 4),
        static_cast<uint8_t>(((kd_uint & 0xF) << 4) | ((tau_uint >> 8) & 0xF)),
        static_cast<uint8_t>(tau_uint & 0xFF)
    };

    sendData(motor.SlaveID, data);
}

void MotorControl::sendData(uint16_t motor_id, const std::array<uint8_t, 8>& data) {
    canbus_.send(motor_id, data);
}

void MotorControl::recv() {
    can_frame frame = canbus_.recv();
    processPacket(frame);           
}

void MotorControl::processPacket(const can_frame& frame) {
    uint16_t motorID = frame.data[0];
    uint8_t cmd = 0x11;  // someday fix

    if (cmd == 0x11) {
        if (motorID != 0x00) {
            auto it = motors_map.find(motorID);
            if (it != motors_map.end() && it->second) { 
                Motor* motor = it->second;

                uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
                uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
                uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
                int t_mos = frame.data[6];
                int t_rotor = frame.data[7];

                double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
                double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
                double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

                double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
                double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
                double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

                motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
            }
        } else {
            uint16_t MasterID = frame.data[0] & 0x0F;
            auto it = motors_map.find(MasterID);
            if (it != motors_map.end() && it->second) { 
                Motor* motor = it->second;

                uint16_t q_uint = (frame.data[1] << 8) | frame.data[2];
                uint16_t dq_uint = (frame.data[3] << 4) | (frame.data[4] >> 4);
                uint16_t tau_uint = ((frame.data[4] & 0xf) << 8) | frame.data[5];
                int t_mos = frame.data[6];
                int t_rotor = frame.data[7];

                double Q_MAX = Limit_Param[static_cast<int>(motor->MotorType)][0];
                double DQ_MAX = Limit_Param[static_cast<int>(motor->MotorType)][1];
                double TAU_MAX = Limit_Param[static_cast<int>(motor->MotorType)][2];

                double recv_q = uint_to_double(q_uint, -Q_MAX, Q_MAX, 16);
                double recv_dq = uint_to_double(dq_uint, -DQ_MAX, DQ_MAX, 12);
                double recv_tau = uint_to_double(tau_uint, -TAU_MAX, TAU_MAX, 12);

                motor->recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor);
            }
        }
    }
}

void MotorControl::controlCmd(Motor& motor, uint8_t cmd) {
    std::array<uint8_t, 8> data_buf = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};
    sendData(motor.SlaveID, data_buf);
}

void MotorControl::readRIDParam(Motor& motor, DM_variable RID) {
    std::array<uint8_t, 8> data = {
        static_cast<uint8_t>(motor.SlaveID & 0xFF),  
        static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF), 
        0x33,  
        static_cast<uint8_t>(RID), 
        0x00, 0x00, 0x00, 0x00 
    };
    canbus_.send(0x7FF, data);
}

void MotorControl::writeMotorParam(Motor& motor, DM_variable RID, double value) {
    std::array<uint8_t, 8> data = {
        static_cast<uint8_t>(motor.SlaveID & 0xFF), 
        static_cast<uint8_t>((motor.SlaveID >> 8) & 0xFF),
        0x55,  
        static_cast<uint8_t>(RID) 
    };

    if (is_in_ranges(static_cast<int>(RID))) {
        auto intData = data_to_uint8s(static_cast<uint32_t>(value));
        std::copy(intData.begin(), intData.end(), data.begin() + 4);
    } else {
        auto doubleData = double_to_uint8s(value);
        std::copy(doubleData.begin(), doubleData.end(), data.begin() + 4);
    }

    canbus_.send(0x7FF, data);
}

