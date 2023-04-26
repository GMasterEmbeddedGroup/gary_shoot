#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/power_heat.hpp"
#include "gary_msgs/srv/switch_barrel.hpp"
#include "std_msgs/msg/float64.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_shoot {

    class ShooterHeatControl : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit ShooterHeatControl(const rclcpp::NodeOptions &options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callbacks
        void trigger_callback(std_msgs::msg::Float64::SharedPtr msg);
        void power_heat_callback(gary_msgs::msg::PowerHeat::SharedPtr msg);

        //service client
        rclcpp::Client<gary_msgs::srv::SwitchBarrel>::SharedPtr switch_barrel_client;

        //params
        std::string trigger_pub_topic;
        std::string trigger_sub_topic;
        std::string power_heat_topic;
        double heat_max_level{};

        //publisher
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr trigger_publisher;

        //subscriber
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr trigger_subscriber;
        rclcpp::Subscription<gary_msgs::msg::PowerHeat>::SharedPtr power_heat_subscriber;

        std::shared_future<gary_msgs::srv::SwitchBarrel::Response::SharedPtr> resp;

        double scale_factor_id1{};
        double scale_factor_id2{};
        bool switching{};
        int barrel_id{};
    };
}
