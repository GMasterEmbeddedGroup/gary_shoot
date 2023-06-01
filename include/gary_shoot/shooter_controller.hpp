#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "gary_msgs/msg/pid.hpp"
#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include <string>
#include <cmath>
#include <chrono>
#include <limits>


static inline bool DoubleEqual(double a, double b)
{
    return std::abs(a - b) < std::numeric_limits<double>::epsilon();
}

namespace gary_shoot{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ShooterController : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ShooterController(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        std_msgs::msg::Float64 LeftShooterWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr LeftShooterWheelPIDPublisher;
        std_msgs::msg::Float64 RightShooterWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr RightShooterWheelPIDPublisher;
        std_msgs::msg::Float64 TriggerWheelPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr TriggerWheelPIDPublisher;
        std_msgs::msg::Float64 TriggerWheelPositionPIDMsg;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr TriggerWheelPositionPIDPublisher;

        void shooter_callback(std_msgs::msg::Float64::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ShooterSubscription;
        void trigger_callback(std_msgs::msg::Float64::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TriggerSubscription;
        void diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr DiagnosticSubscription;
        void pid_callback(gary_msgs::msg::PID::SharedPtr msg);
        rclcpp::Subscription<gary_msgs::msg::PID>::SharedPtr TriggerPIDSubscription;
        void position_pid_callback(gary_msgs::msg::PID::SharedPtr msg);
        rclcpp::Subscription<gary_msgs::msg::PID>::SharedPtr TriggerPositionPIDSubscription;
        void controller_state_callback(controller_manager_msgs::msg::ControllerState::SharedPtr msg);
        rclcpp::Subscription<controller_manager_msgs::msg::ControllerState>::SharedPtr ControllerStateSubscription;

        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;
        std::shared_future<controller_manager_msgs::srv::SwitchController::Response::SharedPtr> resp;

        void data_publisher();
        void reverse_trigger();

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;
        rclcpp::TimerBase::SharedPtr timer_reverse;

        double update_freq;

        std::string shooter_wheel_receive_topic;
        std::string trigger_wheel_receive_topic;
        std::string diagnostic_topic;

        std::string left_wheel_send_topic;
        std::string right_wheel_send_topic;
        double shooter_wheel_pid_target;
        double shooter_wheel_pid_current_set;
        std::string trigger_wheel_send_topic;
        double trigger_wheel_pid_target;
        double trigger_wheel_current_set;

        std::string pid_topic;

        bool shooter_on;
        bool trigger_on;
        bool motor_offline;
        bool zero_force;

        int reverse_time;
        int block_time;
        bool single_shoot;
        bool reverse;
        double reverse_pid_set;
        double real_trigger_speed;
        uint8_t BLOCK_TRIGGER_SPEED_DIFF;
        double real_position;

        int64_t BLOCK_TIME;
        int64_t REVERSE_TIME;
        bool continuously_fire_controller_on;
        bool single_fire_controller_on;
        bool use_single_shoot;
        std::chrono::time_point<std::chrono::steady_clock> last_click_time;
        bool last_trigger_on;
        bool position_changed;

        std::map<std::string,bool> diag_objs;
    };

}
