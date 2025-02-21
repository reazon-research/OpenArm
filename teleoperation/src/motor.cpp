#include "motor.h"

Motor::Motor(DM_Motor_Type motorType, uint16_t slaveID, uint16_t masterID)
    : MotorType(motorType), SlaveID(slaveID), MasterID(masterID),
      Pd(0.0), Vd(0.0), goal_position(0.0), goal_tau(0.0),
      state_q(0.0), state_dq(0.0), state_tau(0.0),
      state_tmos(0), state_trotor(0),
      isEnable(false), NowControlMode(Control_Type::MIT) {}

void Motor::recv_data(double q, double dq, double tau, int tmos, int trotor) {
    state_q = q;
    state_dq = dq;
    state_tau = tau;
    state_tmos = tmos;
    state_trotor = trotor;
}

double Motor::getPosition() const { return state_q; }
double Motor::getVelocity() const { return state_dq; }
double Motor::getTorque() const { return state_tau; }


double Motor::getGoalPosition() const {
    return goal_position;
}

void Motor::setGoalPosition(double pos) {
    goal_position = pos;
}

double Motor::getGoalTau() const {
    return goal_tau;
}

void Motor::setGoalTau(double tau) {
    goal_tau = tau;
}

int Motor::getStateTmos() const {
    return state_tmos;
}

void Motor::setStateTmos(int tmos) {
    state_tmos = tmos;
}

int Motor::getStateTrotor() const {
    return state_trotor;
}

void Motor::setStateTrotor(int trotor) {
    state_trotor = trotor;
}

int Motor::getParam(int RID) const {
    auto it = temp_param_dict.find(RID);
    return (it != temp_param_dict.end()) ? it->second : -1;
}

double LIMIT_MIN_MAX(double x, double min, double max) {
    return std::max(min, std::min(x, max));
}

uint16_t double_to_uint(double x, double x_min, double x_max, int bits) {
    x = LIMIT_MIN_MAX(x, x_min, x_max);
    double span = x_max - x_min;
    double data_norm = (x - x_min) / span;
    return static_cast<uint16_t>(data_norm * ((1 << bits) - 1));
}

double uint_to_double(uint16_t x, double min, double max, int bits) {
    double span = max - min;
    double data_norm = static_cast<double>(x) / ((1 << bits) - 1);
    return data_norm * span + min;
}

std::array<uint8_t, 4> double_to_uint8s(double value) {
    std::array<uint8_t, 4> bytes;
    std::memcpy(bytes.data(), &value, sizeof(double));
    return bytes;
}

std::array<uint8_t, 4> data_to_uint8s(uint32_t value) {
    std::array<uint8_t, 4> bytes;
    std::memcpy(bytes.data(), &value, sizeof(uint32_t));
    return bytes;
}

uint32_t uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    uint32_t value;
    uint8_t bytes[4] = {byte1, byte2, byte3, byte4};
    std::memcpy(&value, bytes, sizeof(uint32_t));
    return value;
}

double uint8s_to_double(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4) {
    double value;
    uint8_t bytes[4] = {byte1, byte2, byte3, byte4};
    std::memcpy(&value, bytes, sizeof(double));
    return value;
}

bool is_in_ranges(int number) {
    return (7 <= number && number <= 10) || (13 <= number && number <= 16) || (35 <= number && number <= 36);
}

void print_hex(const std::vector<uint8_t>& data) {
    for (auto byte : data) {
        std::cout << std::hex << std::uppercase << (int)byte << " ";
    }
    std::cout << std::dec << std::endl;
}

template <typename T>
T get_enum_by_index(int index) {
    if (index >= 0 && index < static_cast<int>(T::COUNT)) {
        return static_cast<T>(index);
    }
    return static_cast<T>(-1);
}
