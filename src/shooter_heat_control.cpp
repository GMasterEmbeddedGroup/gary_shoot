#include "gary_shoot/shooter_heat_control.hpp"


using namespace std::chrono_literals;

using namespace gary_shoot;


ShooterHeatControl::ShooterHeatControl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "shooter_heat_control", options) {
    //declare params
    this->declare_parameter("trigger_pub_topic", "/shooter/rc_trigger_set_limited");
    this->declare_parameter("trigger_sub_topic", "/shooter/rc_trigger_set");
    this->declare_parameter("power_heat_topic", "/referee/power_heat");
    this->declare_parameter("heat_control_level", 150.0f);
    this->declare_parameter("heat_min_level", 50.0f);
}

CallbackReturn ShooterHeatControl::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = this->cb_group;

    //get trigger_pub_topic
    this->trigger_pub_topic = this->get_parameter("trigger_pub_topic").as_string();
    this->trigger_publisher = this->create_publisher<std_msgs::msg::Float64>(this->trigger_pub_topic,
                                                                              rclcpp::SystemDefaultsQoS());
    //get trigger_pub_topic
    this->trigger_sub_topic = this->get_parameter("trigger_sub_topic").as_string();
    this->trigger_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            this->trigger_sub_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::trigger_callback, this, std::placeholders::_1), sub_options);

    //get power_heat_topic
    this->power_heat_topic = this->get_parameter("power_heat_topic").as_string();
    this->power_heat_subscriber = this->create_subscription<gary_msgs::msg::PowerHeat>(
            this->power_heat_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::power_heat_callback, this, std::placeholders::_1), sub_options);

    //get heat_control_level
    this->heat_control_level = this->get_parameter("heat_control_level").as_double();

    //get heat_min_level
    this->heat_min_level = this->get_parameter("heat_min_level").as_double();

    this->scale_factor = 1.0f;

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->trigger_publisher.reset();
    this->trigger_subscriber.reset();
    this->power_heat_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->trigger_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->trigger_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->trigger_publisher.get() != nullptr) this->trigger_publisher.reset();
    if (this->trigger_subscriber.get() != nullptr) this->trigger_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->trigger_publisher.get() != nullptr) this->trigger_publisher.reset();
    if (this->trigger_subscriber.get() != nullptr) this->trigger_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void ShooterHeatControl::trigger_callback(std_msgs::msg::Float64 ::SharedPtr msg) {
    std_msgs::msg::Float64 data;
    data.data = msg->data * this->scale_factor;
    this->trigger_publisher->publish(data);
}


void ShooterHeatControl::power_heat_callback(gary_msgs::msg::PowerHeat::SharedPtr msg) {
    if (static_cast<double>(msg->shooter_17mm_id1_heat) < this->heat_min_level) {
        this->scale_factor = 1.0;
    } else {
        this->scale_factor = (this->heat_control_level - static_cast<double>(msg->shooter_17mm_id1_heat)) /
                             (this->heat_control_level - this->heat_min_level);
    }
    this->scale_factor = std::min<double>(this->scale_factor, 1.0f);
    this->scale_factor = std::max<double>(this->scale_factor, 0.0f);
}


int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<ShooterHeatControl> shooter_heat_control = std::make_shared<ShooterHeatControl>(
            rclcpp::NodeOptions());

    exe.add_node(shooter_heat_control->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::ShooterHeatControl)
