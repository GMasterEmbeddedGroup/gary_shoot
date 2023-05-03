#include "gary_shoot/shooter_heat_control.hpp"


using namespace std::chrono_literals;

using namespace gary_shoot;


ShooterHeatControl::ShooterHeatControl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
        "shooter_heat_control", options) {
    //declare params
    this->declare_parameter("trigger_pub_topic", "/shooter/rc_trigger_set_limited");
    this->declare_parameter("shooter_pub_topic", "/shooter/rc_shooter_set_limited");
    this->declare_parameter("trigger_sub_topic", "/shooter/rc_trigger_set");
    this->declare_parameter("shooter_sub_topic", "/shooter/rc_shooter_set");
    this->declare_parameter("power_heat_topic", "/referee/power_heat");
    this->declare_parameter("heat_max_level", 200.0f);
    this->barrel_max_speed_by_id.resize(3* sizeof(float));
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

    //get shooter_pub_topic
    this->shooter_pub_topic = this->get_parameter("shooter_pub_topic").as_string();
    this->shooter_publisher = this->create_publisher<std_msgs::msg::Float64>(this->shooter_pub_topic,
                                                                              rclcpp::SystemDefaultsQoS());
    //get shooter_pub_topic
    this->shooter_sub_topic = this->get_parameter("shooter_sub_topic").as_string();
    this->shooter_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            this->shooter_sub_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::shooter_callback, this, std::placeholders::_1), sub_options);

    //get power_heat_topic
    this->power_heat_topic = this->get_parameter("power_heat_topic").as_string();
    this->power_heat_subscriber = this->create_subscription<gary_msgs::msg::PowerHeat>(
            this->power_heat_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::power_heat_callback, this, std::placeholders::_1), sub_options);

    this->shoot_data_subscriber = this->create_subscription<gary_msgs::msg::ShootData>(
            "/referee/shoot_data", rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::shoot_data_callback, this, std::placeholders::_1), sub_options);
    this->robot_status_subscriber = this->create_subscription<gary_msgs::msg::RobotStatus>(
            "/referee/robot_status", rclcpp::SystemDefaultsQoS(),
            std::bind(&ShooterHeatControl::robot_status_callback, this, std::placeholders::_1), sub_options);

    //get heat_max_level
    this->heat_max_level = this->get_parameter("heat_max_level").as_double();

    this->switch_barrel_client = this->create_client<gary_msgs::srv::SwitchBarrel>("/switch_barrel");

    this->scale_factor_id1 = 1.0f;
    this->scale_factor_id2 = 1.0f;

    RCLCPP_INFO(this->get_logger(), "configured");

    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->trigger_publisher.reset();
    this->trigger_subscriber.reset();
    this->shooter_publisher.reset();
    this->shooter_subscriber.reset();
    this->power_heat_subscriber.reset();
    if (this->robot_status_subscriber.get() != nullptr) this->robot_status_subscriber.reset();
    if (this->shoot_data_subscriber.get() != nullptr) this->shoot_data_subscriber.reset();


    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->trigger_publisher->on_activate();
    this->shooter_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->trigger_publisher->on_deactivate();
    this->shooter_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->trigger_publisher.get() != nullptr) this->trigger_publisher.reset();
    if (this->trigger_subscriber.get() != nullptr) this->trigger_subscriber.reset();
    if (this->shooter_publisher.get() != nullptr) this->shooter_publisher.reset();
    if (this->shooter_subscriber.get() != nullptr) this->shooter_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();
    if (this->robot_status_subscriber.get() != nullptr) this->robot_status_subscriber.reset();
    if (this->shoot_data_subscriber.get() != nullptr) this->shoot_data_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ShooterHeatControl::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->trigger_publisher.get() != nullptr) this->trigger_publisher.reset();
    if (this->trigger_subscriber.get() != nullptr) this->trigger_subscriber.reset();
    if (this->shooter_publisher.get() != nullptr) this->shooter_publisher.reset();
    if (this->shooter_subscriber.get() != nullptr) this->shooter_subscriber.reset();
    if (this->power_heat_subscriber.get() != nullptr) this->power_heat_subscriber.reset();
    if (this->robot_status_subscriber.get() != nullptr) this->robot_status_subscriber.reset();
    if (this->shoot_data_subscriber.get() != nullptr) this->shoot_data_subscriber.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void ShooterHeatControl::trigger_callback(std_msgs::msg::Float64 ::SharedPtr msg) {
    std_msgs::msg::Float64 data;

    //switcher is not available
    if (! this->switch_barrel_client->service_is_ready()) {
        if (this->scale_factor_id1 == 0.0 || this->scale_factor_id2 == 0.0) {
            data.data = 0.0;
        } else {
            data.data = msg->data;
        }
        this->trigger_publisher->publish(data);
        return;
    }

    //get switch status
    if (this->switching) {
        if (this->resp.wait_for(0ms) == std::future_status::ready) {
            if (resp.get()->success) {
                this->switching = false;
                RCLCPP_INFO(this->get_logger(), "switched to barrel %d", this->barrel_id);
            } else {
                auto req = std::make_shared<gary_msgs::srv::SwitchBarrel::Request>();
                req->barrel_id = this->barrel_id;
                this->resp = this->switch_barrel_client->async_send_request(req);
            }

        }
        data.data = 0.0;
        this->trigger_publisher->publish(data);
        return;
    }

    if (this->barrel_id == 0 && this->scale_factor_id1 == 0.0) {
        auto req = std::make_shared<gary_msgs::srv::SwitchBarrel::Request>();
        req->barrel_id = 1;
        this->barrel_id = 1;
        this->switching = true;
        this->resp = this->switch_barrel_client->async_send_request(req);
        data.data = 0.0;
        RCLCPP_INFO(this->get_logger(), "switching to barrel %d", this->barrel_id);
    } else if (this->barrel_id == 1 && this->scale_factor_id2 == 0.0) {
        auto req = std::make_shared<gary_msgs::srv::SwitchBarrel::Request>();
        req->barrel_id = 0;
        this->barrel_id = 0;
        this->switching = true;
        this->resp = this->switch_barrel_client->async_send_request(req);
        data.data = 0.0;
        RCLCPP_INFO(this->get_logger(), "switching to barrel %d", this->barrel_id);
    } else {
        if (this->barrel_id == 0) data.data = msg->data * this->scale_factor_id1;
        if (this->barrel_id == 1) data.data = msg->data * this->scale_factor_id2;
    }

    this->trigger_publisher->publish(data);
}


void ShooterHeatControl::power_heat_callback(gary_msgs::msg::PowerHeat::SharedPtr msg) {
    this->scale_factor_id1 = (static_cast<double>(msg->shooter_17mm_id1_heat) < this->heat_max_level) ? 1.0 : 0.0;
    this->scale_factor_id2 = (static_cast<double>(msg->shooter_17mm_id2_heat) < this->heat_max_level) ? 1.0 : 0.0;
}

void ShooterHeatControl::shoot_data_callback(gary_msgs::msg::ShootData::SharedPtr msg) {
    if((this->barrel_id == 0 && msg->shooter_id == msg->SHOOTER_ID_17MM_ID1)
    || (this->barrel_id == 1 && msg->shooter_id == msg->SHOOTER_ID_17MM_ID2)){
        this->shoot_speed = msg->bullet_speed;
    }else{
        RCLCPP_WARN(this->get_logger(),"Barrel_ID cannot be matched.");
    }
}

void ShooterHeatControl::shooter_callback(std_msgs::msg::Float64::SharedPtr msg) {
    std_msgs::msg::Float64 data;
    double tmp = 0.0;
    if(this->shoot_speed <= this->barrel_max_speed_by_id[barrel_id]){
        tmp = msg->data;
    }else{
        tmp = msg->data - ((this->shoot_speed - this->barrel_max_speed_by_id[barrel_id]) * 263.1578947368421);
    }
    data.data = (tmp>0.001) ? tmp : 0.0;
    this->shooter_publisher->publish(data);
}

void ShooterHeatControl::robot_status_callback(gary_msgs::msg::RobotStatus::SharedPtr msg) {
    this->barrel_max_speed_by_id[0] = msg->shooter_17mm_id1_speed_limit * 0.8f;
    this->barrel_max_speed_by_id[1] = msg->shooter_17mm_id2_speed_limit * 0.8f;
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
