#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>  
#include "damiao_port.h"
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <threads.h>

#include "control.h"
#include "dynamics.h"
#include "global.h"


class LeaderNode: public rclcpp::Node
{
    Control *control_l_;
    rclcpp::TimerBase::SharedPtr timer_;

    public:
    LeaderNode(Control *control_l): Node("LeaderNode"), control_l_(control_l)
    {
      double Ts = 1.0 / FREQUENCY;
      int msec = (int) (Ts * 1000);
  
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(msec),
        std::bind(&LeaderNode::timer_callback, this));
  
      RCLCPP_INFO(this->get_logger(), "LeaderNode initialized");
    }
  
    ~LeaderNode()
    {
      control_l_->Shutdown();
    }

    private:
    void timer_callback()
    {
      //control_l_->DoControl();
      control_l_->DoControl_u();

    }
};
  
class FollowerNode: public rclcpp::Node
{
  Control *control_f_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  FollowerNode(Control *control_f) : Node("FollowerNode"), control_f_(control_f)
  {
    double Ts = 1.0 / FREQUENCY;
    int msec = (int) (Ts * 1000);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(msec),
      std::bind(&FollowerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "FollowerNode initialized");
  }

  ~FollowerNode()
  {
    control_f_->Shutdown();
  }

private:
  void timer_callback()
  {
    //control_f_->DoControl();
    control_f_->DoControl_u();
    
  }
};



class AdminNode: public rclcpp::Node
{
    Control *control_l_;
    Control *control_f_;
  
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::JointState response_l;
    sensor_msgs::msg::JointState response_f;
  
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_l_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_f_;

    public:
        AdminNode(Control *control_l, Control *control_f) : Node("AdminNode"), control_l_(control_l), control_f_(control_f)
        {
        setup_all_publishers();
    
        double Ts = 1.0 / FREQUENCY;
        int msec = (int) (Ts * 1000);
    
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(msec),
            std::bind(&AdminNode::timer_callback, this));
    
        RCLCPP_INFO(this->get_logger(), "AdminNode initialized");
        }

        ~AdminNode()
        {
        }
    private:
    void setup_all_publishers(void)
    {
        pub_l_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "openarm/leader/joint_state", 1
        );
        pub_f_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "openarm/follower/joint_state", 1
        );
    }

    void timer_callback()
    {

      control_f_->GetResponse(&response_f);
      control_l_->GetResponse(&response_l);
  
      for (int i = 0; i < NJOINTS; ++i) {
        response_f.effort[i] *= BETA;
        response_l.position[i] *= ALPHA;
      }
  
      control_f_->SetReference(&response_l);
      control_l_->SetReference(&response_f);
  
    //   control_l_->Debug();
  
      pub_l_->publish(*control_l_->response_);
      pub_f_->publish(*control_f_->response_);

  
    }

  };




int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    DamiaoPort* arm_l = new DamiaoPort(LEADER_DEVICENAME0, 
        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
        DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
        {true, true, true, true, true, true, true, true},
        Control_Type::MIT);

    DamiaoPort* arm_f = new DamiaoPort(FOLLOWER_DEVICENAME0, 
        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
         DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
        {true, true, true, true, true, true, true, true},
        Control_Type::MIT);

    arm_l->setZeroPosition();
    arm_f->setZeroPosition();

    Control* control_l = new Control(arm_l, 1.0 / FREQUENCY, ROLE_LEADER);
    Control* control_f = new Control(arm_f, 1.0 / FREQUENCY, ROLE_FOLLOWER);
        
    control_l->Configure(Dn_L, Gn_L, Jn_L, gn_L, Kp_L, Kd_L, Kf_L);
    control_f->Configure(Dn_F, Gn_F, Jn_F, gn_F, Kp_F, Kd_F, Kf_F);

    if (!control_l->Setup()) {
        return 1;
    
    }
    if (!control_f->Setup()) {
        return 1;
    }
    
    std::vector<double> init_pos = {0, 0, 0, 0, 0, 0, 0};
    std::vector<double> kp = {2.0,2.0,2.0,2.0,2.0,2.0,2.0};
    std::vector<double> kd = {0.1,0.1,0.1,0.1,0.1,0.1,0.1};


    arm_l->setGoalPositionsSync(init_pos, kp, kd);
    arm_f->setGoalPositionsSync(init_pos, kp, kd);

    std::vector<double> positions_l = arm_l->getPositions();
    std::vector<double> velocities_l = arm_l->getVelocities();

    std::vector<double> positions_f = arm_f->getPositions();
    std::vector<double> velocities_f = arm_f->getVelocities();

    for(int i = 0; i < NMOTORS ; i++)
    {
        control_l->response_->position[i] = positions_l[i];
        control_f->response_->position[i] = positions_f[i];
    }

    sleep(1);

    control_l->SetReference(control_f->response_);
    control_f->SetReference(control_l->response_);

    //set home postion
    std::thread thread_l(&Control::AdjustPosition, control_l);
    std::thread thread_f(&Control::AdjustPosition, control_f);
    
    thread_l.join();
    thread_f.join();

    control_l->SetReference(control_f->response_);
    control_f->SetReference(control_l->response_);

    rclcpp::executors::MultiThreadedExecutor exec;
    auto leader_node = std::make_shared<LeaderNode>(control_l);
    auto follower_node = std::make_shared<FollowerNode>(control_f);
    auto admin_node = std::make_shared<AdminNode>(control_l, control_f);

    exec.add_node(leader_node);
    exec.add_node(follower_node);
    exec.add_node(admin_node);
  
    exec.spin();

    rclcpp::shutdown();
    return 0;

}


