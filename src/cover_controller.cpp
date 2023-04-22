#include "gary_shoot/cover_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <chrono>
#include <cmath>

namespace gary_shoot {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CoverController::CoverController(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "cover_controller", options) {
        this->CoverSwitchService = nullptr;
        SwitcherEffortPIDPublisher = nullptr;
//        this->ControllerSwitchClient = nullptr;
        this->timer_pub = nullptr;
        switched = false;
        switching = false;
        this->declare_parameter("effort_max", 5.0);
        this->declare_parameter("effort_min", 0.2);
        effort_max = 5.0;
        effort_min = 0.2;
        current_effort = effort_min;

    }

    CallbackReturn CoverController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        this->effort_max =  this->get_parameter("effort_max").as_double();
        this->effort_min =  this->get_parameter("effort_min").as_double();
        current_effort = effort_min;

        SwitcherEffortPIDPublisher =
                create_publisher<std_msgs::msg::Float64>
                        ("/cover_switch_pid_effort/cmd", rclcpp::SystemDefaultsQoS());
        CoverSwitchService =
                create_service<gary_msgs::srv::SwitchCover>("/switch_cover",
                                                            std::bind(&CoverController::switch_srv_callback, this,
                                                                      std::placeholders::_1, std::placeholders::_2));

//        this->need_zero = true;

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherEffortPIDPublisher.get() != nullptr) SwitcherEffortPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        SwitcherEffortPIDMsg.data = 0;

        SwitcherEffortPIDPublisher->on_activate();

        this->timer_pub = this->create_wall_timer(1000ms / 100, [this] { publisher(); });

        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherEffortPIDPublisher.get() != nullptr) SwitcherEffortPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherEffortPIDPublisher.get() != nullptr) SwitcherEffortPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherEffortPIDPublisher.get() != nullptr) SwitcherEffortPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void CoverController::switch_srv_callback(const std::shared_ptr<gary_msgs::srv::SwitchCover::Request> request,
                                              std::shared_ptr<gary_msgs::srv::SwitchCover::Response> response) {

        RCLCPP_INFO(this->get_logger(), "Received Call.");

        if (request->open) {
            if (switched) {
                current_effort = effort_max;
                switching = true;
                RCLCPP_INFO(this->get_logger(), "Switching to Cover 0...");
            }
            switched = false;
        } else if (!request->open) {
            if (!switched) {
                current_effort = effort_max;
                switching = true;
                RCLCPP_INFO(this->get_logger(), "Switching to Cover 1...");
            }
            switched = true;
        }

        response->success = !switching;

    }

    void CoverController::publisher() {
        static int state = 0;
        static double delay_time = DELAY_TIME;
        static double switch_time = SWITCH_TIME;
        if(switching){
            if(state == 0){
                delay_time -= 10.0;
                if(delay_time <= 0.0){
                    state = 1;
                    delay_time = DELAY_TIME;
                    RCLCPP_INFO(this->get_logger(),"Pre-Delayed...");
                    return;
                }
                return;
            }else if(state == 1){
                current_effort = effort_max;
                switch_time -= 10.0;
                if(switch_time <= 0.0){
                    state = 2;
                    switch_time = SWITCH_TIME;
                    RCLCPP_INFO(this->get_logger(),"Switched!");
                    return;
                }
                return;
            }else if (state == 2){
                current_effort = effort_min;
                delay_time -= 10.0;
                if(delay_time <= 0.0){
                    state = 0;
                    delay_time = DELAY_TIME;
                    switching = false;
                    RCLCPP_INFO(this->get_logger(),"Past-Delayed...");
                    return;
                }
                return;
            }
        }
        if (!switched) {
            SwitcherEffortPIDMsg.data = 0 + current_effort;
        } else {
            SwitcherEffortPIDMsg.data = 0 - current_effort;
        }
        auto clock = rclcpp::Clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "setting: %lf", SwitcherEffortPIDMsg.data);
        if (SwitcherEffortPIDPublisher.get() != nullptr) SwitcherEffortPIDPublisher->publish(SwitcherEffortPIDMsg);
    }
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::CoverController> coverController =
            std::make_shared<gary_shoot::CoverController>(rclcpp::NodeOptions());

    exe.add_node(coverController->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::CoverController)