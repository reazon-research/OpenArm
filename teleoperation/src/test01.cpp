#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>  
#include "damiao_port.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp> 
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#define FOLLOWER_DEVICENAME0 "can1"
#define TICK 0.02
#define DOF 7

using namespace KDL;

volatile bool running = true;


void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting...\n";
    running = false;  
}

Tree kdlTreeFromURDFModel(const std::string& urdf_path) {
    urdf::Model urdf_model;
    
    if (!urdf_model.initFile(urdf_path)) {
        std::cerr << "Failed to load URDF file: " << urdf_path << std::endl;
        exit(EXIT_FAILURE);
    }

    Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
        std::cerr << "Failed to parse URDF into KDL Tree." << std::endl;
        exit(EXIT_FAILURE);
    }

    return tree;
}

int main() {
    // Ctrl+C (SIGINT) をキャッチするための設定
    signal(SIGINT, signalHandler);
    std::string urdf_path = "/home/toki/ros2_ws/src/openarm_bilateral/urdf/openarm_grip.urdf";

    Tree kdl_tree = kdlTreeFromURDFModel(urdf_path);
    printf("tree get !!!!!\n\r");

    Chain chain;
    if (!kdl_tree.getChain("base_link", "grip_attach_1", chain)) {
        std::cerr << "Failed to extract chain from KDL Tree." << std::endl;
        return -1;
    }
    printf("chain get !!!!!\n\r");

    std::cout << "Chain Number of joints: " << chain.getNrOfJoints() << std::endl;

    Vector grav_vector(0, 0, 9.81);

    // ChainDynParamを使った動力学計算用のオブジェクトを作成
    std::unique_ptr<KDL::ChainDynParam> dyn_param_solver;
    dyn_param_solver = std::make_unique<KDL::ChainDynParam>(chain, grav_vector);

    DamiaoPort follower(FOLLOWER_DEVICENAME0, 
                        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
                         DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},
                        {true,true,true,true,true,true,true},
                        Control_Type::MIT);

    follower.moveTorqueSync({0, 0, 0, 0, 0, 0, 0});
    auto t0 = std::chrono::steady_clock::now();
    auto t_schedule = t0;
    double k = 0.8;
    std::vector<double> biases = {0, -1.4, 0, -2.7, 0, 0, 0};

    while (running) {  
        auto loop_start_time = std::chrono::steady_clock::now();
    
        JntArray jt_positions(DOF);
        std::vector<double> pose;
        std::vector<double> positions = follower.getPositions();

        for (size_t i = 0; i < follower.getMotors().size(); ++i) {
            // float pos = follower.getMotors()[i]->getPosition() + biases[i];
            // float vel = follower.getMotors()[i]->getVelocity();
            // // std::cout << "vel : " << vel << std::endl;
            // pose.push_back(pos);
            jt_positions(i) = positions[i] + biases[i];
            std::cout << "jt_positions : " << jt_positions(i) << std::endl;
        }
        std::cout << "============================================" <<std::endl;

        // 重力によるトルクを計算
        JntArray grav_torque(DOF);
        dyn_param_solver->JntToGravity(jt_positions, grav_torque);
    
        std::vector<double> res;
        for (size_t i = 0; i < DOF; ++i) {
            res.push_back(k * grav_torque(i));
        }

        follower.moveTorqueSync2(res);

        std::vector<double> init_pos = {0, 0, 0, 0, 0, 0, 0};
        std::vector<double> kp = {2.0,2.0,2.0,2.0,2.0,2.0,2.0};
        std::vector<double> kd = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};
        // follower.setGoalPositionsSync(init_pos, kp, kd);
    
        // **ループ終了時刻を取得し、制御周期を計測**
        auto loop_end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> loop_duration = loop_end_time - loop_start_time;
        // std::cout << "Loop execution time: " << loop_duration.count()*1000 << " ms" << std::endl;
    
        // 次のスケジュール時間を更新
        t_schedule += std::chrono::milliseconds(static_cast<int>(TICK * 1000));
    
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> td = t_schedule - now;
    
        if (td.count() > 0) {
            std::this_thread::sleep_for(td);
        } else {
            std::cerr << "timeout " << td.count() << " sec" << std::endl;
        }
    }

    // ループを抜けた後の処理
    follower.disable();
    std::cout << "done" << std::endl;
    return 0;
}
