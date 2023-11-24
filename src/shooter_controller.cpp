#include "gary_shoot/shooter_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <string>
#include <chrono>

using namespace std::chrono_literals;

namespace gary_shoot {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    ShooterController::ShooterController(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "shooter_controller", options) {
        this->declare_parameter("update_freq", 100.0f);
        this->declare_parameter("shooter_wheel_receive_topic", "/shooter_wheel_controller_pid_set");
        this->declare_parameter("trigger_wheel_receive_topic", "/trigger_wheel_controller_pid_set");
        this->declare_parameter("left_shooter_wheel_send_topic", "/fric_left_pid/cmd");
        this->declare_parameter("right_shooter_wheel_send_topic", "/fric_right_pid/cmd");
        this->declare_parameter("trigger_wheel_send_topic", "/trigger_pid/cmd");
        this->declare_parameter("left_shooter_wheel_diag_name", "fric_left");
        this->declare_parameter("right_shooter_wheel_diag_name", "fric_right");
        this->declare_parameter("trigger_wheel_diag_name", "trigger");
        this->declare_parameter("diagnostic_topic", "/diagnostics_agg");
        this->declare_parameter("pid_topic", "/trigger_pid/pid");
        this->declare_parameter("reverse_pid_set", -3000.0);
        this->declare_parameter("block_time_ms", 500);
        this->declare_parameter("reverse_time_ms", 500);
        this->declare_parameter("single_shoot", true);

        this->declare_parameter("left_back_shooter_wheel_send_topic", "/fric_left_back_pid/cmd");
        this->declare_parameter("right_back_shooter_wheel_send_topic", "/fric_right_back_pid/cmd");

        back_shooter_wheel_pid_target = 0.0;
        back_shooter_wheel_pid_current_set = 0.0;

        this->shooter_wheel_receive_topic = "/shooter_wheel_controller_pid_set";
        this->trigger_wheel_receive_topic = "/trigger_wheel_controller_pid_set";
        this->left_wheel_send_topic = "/fric_left_pid/cmd";
        this->right_wheel_send_topic = "/fric_right_pid/cmd";
        this->left_back_wheel_send_topic = "/fric_left_back_pid/cmd";
        this->right_back_wheel_send_topic = "/fric_right_back_pid/cmd";
        this->shooter_wheel_pid_target = static_cast<double>(0.0f);
        this->trigger_wheel_send_topic = "/trigger_pid/cmd";
        this->trigger_wheel_pid_target = static_cast<double>(0.0f);

        this->diagnostic_topic = "/diagnostics_agg";
        this->motor_offline = true;

        this->update_freq = 100.0f;
        this->shooter_on = false;
        this->trigger_on = false;
        this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
        this->trigger_wheel_current_set = static_cast<double>(0.0f);

        zero_force = true;

        reverse_time = 0;
        block_time = 0;
        reverse = false;
        reverse_pid_set = -3000.0;
        BLOCK_TRIGGER_SPEED_DIFF = 100;
        BLOCK_TIME = 500;
        REVERSE_TIME = 500;

        continuously_fire_controller_on = false;
        single_fire_controller_on = false;
        use_single_shoot = true;
        last_trigger_on = false;
        position_changed = false;

        diag_objs.clear();
        real_trigger_speed = trigger_wheel_current_set;
    }

    CallbackReturn ShooterController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        //create callback group
        this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group;

        this->update_freq = this->get_parameter("update_freq").as_double();

        left_wheel_send_topic = this->get_parameter("left_shooter_wheel_send_topic").as_string();
        LeftShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(left_wheel_send_topic, rclcpp::SystemDefaultsQoS());

        right_wheel_send_topic = this->get_parameter("right_shooter_wheel_send_topic").as_string();
        RightShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(right_wheel_send_topic, rclcpp::SystemDefaultsQoS());

        trigger_wheel_send_topic = this->get_parameter("trigger_wheel_send_topic").as_string();
        TriggerWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(trigger_wheel_send_topic, rclcpp::SystemDefaultsQoS());
        TriggerWheelPositionPIDPublisher =
                create_publisher<std_msgs::msg::Float64>("/trigger_position_pid/cmd", rclcpp::SystemDefaultsQoS());


        shooter_wheel_receive_topic = this->get_parameter("shooter_wheel_receive_topic").as_string();
        this->ShooterSubscription =
                this->create_subscription<std_msgs::msg::Float64>(
                        shooter_wheel_receive_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&ShooterController::shooter_callback, this, std::placeholders::_1), sub_options);


        trigger_wheel_receive_topic = this->get_parameter("trigger_wheel_receive_topic").as_string();
        this->TriggerSubscription =
                this->create_subscription<std_msgs::msg::Float64>(
                        trigger_wheel_receive_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&ShooterController::trigger_callback, this, std::placeholders::_1), sub_options);

        diagnostic_topic = this->get_parameter("diagnostic_topic").as_string();
        this->DiagnosticSubscription =
                this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
                        diagnostic_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&ShooterController::diag_callback, this, std::placeholders::_1), sub_options);

        pid_topic = this->get_parameter("pid_topic").as_string();
        this->TriggerPIDSubscription =
                this->create_subscription<gary_msgs::msg::PID>(
                        pid_topic,
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&ShooterController::pid_callback, this, std::placeholders::_1), sub_options);
        this->TriggerPositionPIDSubscription =
                this->create_subscription<gary_msgs::msg::PID>(
                        "/trigger_position_pid/pid",
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&ShooterController::position_pid_callback, this, std::placeholders::_1), sub_options);


        left_back_wheel_send_topic = this->get_parameter("left_back_shooter_wheel_send_topic").as_string();
        LeftBackShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(left_back_wheel_send_topic, rclcpp::SystemDefaultsQoS());

        right_back_wheel_send_topic = this->get_parameter("right_back_shooter_wheel_send_topic").as_string();
        RightBackShooterWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>(right_back_wheel_send_topic, rclcpp::SystemDefaultsQoS());

        SubTriggerWheelPIDPublisher =
                create_publisher<std_msgs::msg::Float64>("/sub_trigger_pid/cmd", rclcpp::SystemDefaultsQoS());


        reverse_pid_set = this->get_parameter("reverse_pid_set").as_double();
        diag_objs.emplace(this->get_parameter("trigger_wheel_diag_name").as_string(), false);
        diag_objs.emplace(this->get_parameter("left_shooter_wheel_diag_name").as_string(), false);
        diag_objs.emplace(this->get_parameter("right_shooter_wheel_diag_name").as_string(), false);
        diag_objs.emplace("sub_trigger", false);
        diag_objs.emplace("fric_right_back", false);
        diag_objs.emplace("fric_left_back", false);

        BLOCK_TIME = this->get_parameter("block_time_ms").as_int();
        REVERSE_TIME = this->get_parameter("reverse_time_ms").as_int();
        use_single_shoot = this->get_parameter("single_shoot").as_bool();
        if(use_single_shoot){
            this->switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>
                    ("/controller_manager/switch_controller");
            this->list_controllers_client = this->create_client<controller_manager_msgs::srv::ListControllers>
                    ("/controller_manager/list_controllers");
        }

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (timer_update.get() != nullptr) timer_update->reset();
        if (timer_reverse.get() != nullptr) timer_reverse->reset();
        if (TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if (SubTriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightBackShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftBackShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if (TriggerSubscription.get() != nullptr) TriggerSubscription.reset();
        if (ShooterSubscription.get() != nullptr) ShooterSubscription.reset();
        if (TriggerPIDSubscription.get() != nullptr) TriggerPIDSubscription.reset();

        diag_objs.clear();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        timer_update = this->create_wall_timer(1000ms / this->update_freq, [this] { data_publisher(); }, this->cb_group);
        timer_reverse = this->create_wall_timer(1000ms / 100.0, [this] { reverse_trigger(); }, this->cb_group);
        TriggerWheelPIDPublisher->on_activate();
        SubTriggerWheelPIDPublisher->on_activate();
        RightShooterWheelPIDPublisher->on_activate();
        LeftShooterWheelPIDPublisher->on_activate();
        RightBackShooterWheelPIDPublisher->on_activate();
        LeftBackShooterWheelPIDPublisher->on_activate();
        TriggerWheelPositionPIDPublisher->on_activate();

        this->last_click_time = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        timer_update->reset();
        timer_reverse->reset();
        TriggerWheelPIDPublisher->on_deactivate();
        RightShooterWheelPIDPublisher->on_deactivate();
        LeftShooterWheelPIDPublisher->on_deactivate();
        if(TriggerWheelPositionPIDPublisher.get()!= nullptr){TriggerWheelPositionPIDPublisher->on_deactivate();}
        if(TriggerPositionPIDSubscription.get() != nullptr){TriggerPositionPIDSubscription.reset();}
        if(list_controllers_client.get()!= nullptr){list_controllers_client.reset();}
        if(switch_controller_client.get()!= nullptr){switch_controller_client.reset();}
        if (SubTriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightBackShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftBackShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        TriggerSubscription.reset();
        ShooterSubscription.reset();
        TriggerPIDSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (timer_update.get() != nullptr) timer_update->reset();
        if (timer_reverse.get() != nullptr) timer_reverse->reset();
        if (TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if (TriggerSubscription.get() != nullptr) TriggerSubscription.reset();
        if (ShooterSubscription.get() != nullptr) ShooterSubscription.reset();
        if (TriggerPIDSubscription.get() != nullptr) TriggerPIDSubscription.reset();

        if (SubTriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightBackShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftBackShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();

        if(TriggerWheelPositionPIDPublisher.get()!= nullptr){TriggerWheelPositionPIDPublisher->on_deactivate();}
        if(TriggerPositionPIDSubscription.get() != nullptr){TriggerPositionPIDSubscription.reset();}
        if(list_controllers_client.get()!= nullptr){list_controllers_client.reset();}
        if(switch_controller_client.get()!= nullptr){switch_controller_client.reset();}

        diag_objs.clear();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ShooterController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (timer_update.get() != nullptr) timer_update->reset();
        if (timer_reverse.get() != nullptr) timer_reverse->reset();
        if (TriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();
        if (TriggerSubscription.get() != nullptr) TriggerSubscription.reset();
        if (ShooterSubscription.get() != nullptr) ShooterSubscription.reset();
        if (TriggerPIDSubscription.get() != nullptr) TriggerPIDSubscription.reset();

        if (SubTriggerWheelPIDPublisher.get() != nullptr) TriggerWheelPIDPublisher.reset();
        if (RightBackShooterWheelPIDPublisher.get() != nullptr) RightShooterWheelPIDPublisher.reset();
        if (LeftBackShooterWheelPIDPublisher.get() != nullptr) LeftShooterWheelPIDPublisher.reset();

        if(TriggerWheelPositionPIDPublisher.get()!= nullptr){TriggerWheelPositionPIDPublisher->on_deactivate();}
        if(TriggerPositionPIDSubscription.get() != nullptr){TriggerPositionPIDSubscription.reset();}
        if(list_controllers_client.get()!= nullptr){list_controllers_client.reset();}
        if(switch_controller_client.get()!= nullptr){switch_controller_client.reset();}

        diag_objs.clear();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void ShooterController::data_publisher() {
        static uint8_t zero_force_time_cnt = 0;
        static bool offline_warned = false;
        if (!motor_offline) {
            if (this->trigger_on && this->shooter_on) {
                this->trigger_wheel_current_set = (reverse ? this->reverse_pid_set : this->trigger_wheel_pid_target);
                this->sub_trigger_wheel_current_set = (reverse ? 0.0 : this->sub_trigger_wheel_pid_target);
            } else {
                this->trigger_wheel_current_set = static_cast<double>(0.0f);
                this->sub_trigger_wheel_current_set = static_cast<double>(0.0f);
            }
            if (this->shooter_on) {
                if (this->shooter_wheel_pid_current_set < this->shooter_wheel_pid_target) {
                    this->shooter_wheel_pid_current_set += (((1000 / this->update_freq) / 5000) *
                                                            this->shooter_wheel_pid_target);
                } else {
                    this->shooter_wheel_pid_current_set = this->shooter_wheel_pid_target;
                }
                if (this->back_shooter_wheel_pid_current_set < this->back_shooter_wheel_pid_target) {
                    this->back_shooter_wheel_pid_current_set += (((1000 / this->update_freq) / (5000 * abs(shooter_scale))) *
                                                            this->back_shooter_wheel_pid_target);
                } else {
                    this->back_shooter_wheel_pid_current_set = this->back_shooter_wheel_pid_target;
                }
            } else {
                this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
                this->back_shooter_wheel_pid_current_set = 0.0;
            }
            if (offline_warned) {
                RCLCPP_INFO(this->get_logger(), "Motor reconnected");
            }
            offline_warned = false;
        } else {
            if (!offline_warned) {
                RCLCPP_WARN(this->get_logger(), "Shooter shut down due to motor offline.");
                offline_warned = true;
            }
            this->trigger_wheel_current_set = static_cast<double>(0.0f);
            this->shooter_wheel_pid_current_set = static_cast<double>(0.0f);
            this->back_shooter_wheel_pid_current_set = 0.0;
        }

        this->LeftShooterWheelPIDMsg.data = (0 - shooter_wheel_pid_current_set);
        this->RightShooterWheelPIDMsg.data = shooter_wheel_pid_current_set;
        this->LeftBackShooterWheelPIDMsg.data = (0 - back_shooter_wheel_pid_current_set);
        this->RightBackShooterWheelPIDMsg.data = back_shooter_wheel_pid_current_set;
        this->TriggerWheelPIDMsg.data = trigger_wheel_current_set;
        this->SubTriggerWheelPIDMsg.data = sub_trigger_wheel_current_set;
        auto clock = rclcpp::Clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 500, "L:%lf, R:%lf, P:%lf", this->LeftShooterWheelPIDMsg.data,
                              this->RightShooterWheelPIDMsg.data, this->TriggerWheelPIDMsg.data);

        if (!(this->trigger_on || this->shooter_on)) {
            if (zero_force_time_cnt < 20) { zero_force_time_cnt++; }
            else { zero_force = true; }
        } else {
            zero_force = false;
            zero_force_time_cnt = 0;
        }

        if (!zero_force) {
            LeftShooterWheelPIDPublisher->publish(LeftShooterWheelPIDMsg);
            RightShooterWheelPIDPublisher->publish(RightShooterWheelPIDMsg);
            LeftBackShooterWheelPIDPublisher->publish(LeftBackShooterWheelPIDMsg);
            RightBackShooterWheelPIDPublisher->publish(RightBackShooterWheelPIDMsg);
            if(trigger_on) {
                TriggerWheelPIDPublisher->publish(TriggerWheelPIDMsg);
                SubTriggerWheelPIDPublisher->publish(SubTriggerWheelPIDMsg);
            }
        }
    }

    void ShooterController::shooter_callback(std_msgs::msg::Float64::SharedPtr msg) {
        if (!DoubleEqual(msg->data, 0.0)) {
            this->shooter_wheel_pid_target = msg->data;
            this->back_shooter_wheel_pid_target = this->shooter_wheel_pid_target * shooter_scale;
            this->shooter_on = true;
        } else if (DoubleEqual(msg->data, 0.0)) {
            this->shooter_on = false;
        }
    }

    void ShooterController::trigger_callback(std_msgs::msg::Float64::SharedPtr msg) {
        if (!DoubleEqual(msg->data, 0.0)) {
            this->trigger_wheel_pid_target = msg->data;
            this->sub_trigger_wheel_pid_target = this->trigger_wheel_pid_target * sub_trigger_scale;
            this->trigger_on = true;
        } else if (DoubleEqual(msg->data, 0.0)) {
            this->trigger_on = false;
        }
    }

    void ShooterController::diag_callback(diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        for (auto &i: diag_objs) {
            i.second = false;
        }
        for (const auto &status: msg->status) {
            if (status.level != status.OK) {
                if (diag_objs.find(status.name) != diag_objs.end()) {
                    if (!motor_offline) {
                        RCLCPP_ERROR(this->get_logger(), "[%s] on status ERROR!", status.name.c_str());
                    }
                    diag_objs[status.name] = false;
                }
            } else {
                if (diag_objs.find(status.name) != diag_objs.end()) {
                    diag_objs[status.name] = true;
                }
            }
        }
        bool online = true;
        for (const auto &i: diag_objs) {
            online &= i.second;
        }
        motor_offline = !online;
    }

    void ShooterController::pid_callback(gary_msgs::msg::PID::SharedPtr msg) {
        real_trigger_speed = msg->feedback;
    }

    void ShooterController::reverse_trigger() {
//        if(single_fire_controller_on){
//            return;
//        }
        if (shooter_on && trigger_on) {
            if (this->block_time < BLOCK_TIME) {
                reverse = false;
            } else {
                reverse = true;
            }


            auto clock = rclcpp::Clock();

            if (std::abs(real_trigger_speed) < std::abs(trigger_wheel_current_set * 0.5) &&
                this->block_time < BLOCK_TIME) {
                this->block_time += 10;
                this->reverse_time = 0;
                RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "Trigger Blocked.");
            } else if (this->block_time >= BLOCK_TIME && this->reverse_time < REVERSE_TIME) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 1000, "Reversing Trigger...");
                this->reverse_time += 10;
            } else {
                this->block_time = 0;
            }

//            RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 100,
//                                 "Re: real:%lf, cur:%lf, block: %d, rev:%d, ifrev:%d.",
//                                 real_trigger_speed, trigger_wheel_current_set, block_time, reverse_time, reverse);

        } else {
            reverse = false;
        }
    }

    void ShooterController::position_pid_callback(gary_msgs::msg::PID::SharedPtr msg) {
        this->real_position = msg->feedback;
    }

}


int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::ShooterController> shooterController =
            std::make_shared<gary_shoot::ShooterController>(rclcpp::NodeOptions());

    exe.add_node(shooterController->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::ShooterController)