#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/pid.hpp"
#include "gary_msgs/srv/switch_barrel.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "gary_msgs/srv/reset_motor_position.hpp"
#include <string>
#include <cmath>
#include <chrono>
#include <limits>

namespace gary_shoot{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class BarrelSwitcher : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit BarrelSwitcher(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        std_msgs::msg::Float64 SwitcherEffortPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr SwitcherEffortPIDPublisher;
        std_msgs::msg::Float64 SwitcherPositionPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr SwitcherPositionPIDPublisher;

        rclcpp::Service<gary_msgs::srv::SwitchBarrel>::SharedPtr BarrelSwitchService;
        void switch_srv_callback(const std::shared_ptr<gary_msgs::srv::SwitchBarrel::Request> request,
                 std::shared_ptr<gary_msgs::srv::SwitchBarrel::Response> response);

//        void pid_callback(gary_msgs::msg::PID::SharedPtr msg);
//        rclcpp::Subscription<gary_msgs::msg::PID>::SharedPtr SwitcherPIDSubscription;

//        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr ControllerSwitchClient;

        rclcpp::TimerBase::SharedPtr timer_pub;
        void publisher();

        rclcpp::Client<gary_msgs::srv::ResetMotorPosition>::SharedPtr ResetMotorPositionClient;
        bool switching;
        bool switched;
        double effort_max;
        double effort_min;
        volatile double current_effort;

        double SWITCH_TIME;
        double DELAY_TIME;
//        void setzero();

//        double tolerable_diff;
//        int barrel_id;
//        std::map<int,double> position;
//        double current_position;
//        double init_effort;
//        bool need_zero;
    };

}
