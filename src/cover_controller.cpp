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
//        SwitcherEffortPIDPublisher = nullptr;
//        this->ControllerSwitchClient = nullptr;
        this->timer_pub = nullptr;
        this->target_position = 0.0;
        this->open = false;
//        switched = false;
//        switching = false;
//        this->declare_parameter("effort_max", 3.0);
//        this->declare_parameter("effort_min", 0.2);
//        this->declare_parameter("switch_time_ms", 850.0);
//        this->declare_parameter("delay_time_ms", 10.0);
        this->declare_parameter("target_position", 90.0);
        this->target_position = 90.0;
//        effort_max = 3.0;
//        effort_min = 0.2;
//        current_effort = effort_min;

//        SWITCH_TIME = 850.0;
//        DELAY_TIME = 10.0;

    }

    CallbackReturn CoverController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

//        this->effort_max =  this->get_parameter("effort_max").as_double();
//        this->effort_min =  this->get_parameter("effort_min").as_double();
//        this->SWITCH_TIME =  this->get_parameter("switch_time_ms").as_double();
//        this->DELAY_TIME =  this->get_parameter("delay_time_ms").as_double();
        this->target_position =  this->get_parameter("target_position").as_double();
//        current_effort = effort_min;

        SwitcherPositionPIDPublisher =
                create_publisher<std_msgs::msg::Float64>
                        ("/cover_switch_pid_position/cmd", rclcpp::SystemDefaultsQoS());
        CoverSwitchService =
                create_service<gary_msgs::srv::SwitchCover>("/switch_cover",
                                                            std::bind(&CoverController::switch_srv_callback, this,
                                                                      std::placeholders::_1, std::placeholders::_2));

        ResetMotorPositionClient = create_client<gary_msgs::srv::ResetMotorPosition>("/reset_motor_position");
//        this->need_zero = true;

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherPositionPIDPublisher.get() != nullptr) SwitcherPositionPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        SwitcherPositionPIDMsg.data = 0;

        SwitcherPositionPIDPublisher->on_activate();

        if(!reset_motor()){
            return CallbackReturn::FAILURE;
        }

        this->timer_pub = this->create_wall_timer(1000ms / 100, [this] { publisher(); });

        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherPositionPIDPublisher.get() != nullptr) SwitcherPositionPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();
        if(this->timer_pub.get()!= nullptr) this->timer_pub.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherPositionPIDPublisher.get() != nullptr) SwitcherPositionPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CoverController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if (this->SwitcherPositionPIDPublisher.get() != nullptr) SwitcherPositionPIDPublisher.reset();
        if (CoverSwitchService.get() != nullptr) CoverSwitchService.reset();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void CoverController::switch_srv_callback(const std::shared_ptr<gary_msgs::srv::SwitchCover::Request> request,
                                              std::shared_ptr<gary_msgs::srv::SwitchCover::Response> response) {

        RCLCPP_INFO(this->get_logger(), "Received Call.");

        this->open = request->open;

        response->success = true;

    }

    void CoverController::publisher() {
        SwitcherPositionPIDMsg.data = open ? target_position : 0.0;

        auto clock = rclcpp::Clock();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "setting: %lf", SwitcherPositionPIDMsg.data);
        if (SwitcherPositionPIDPublisher.get() != nullptr) SwitcherPositionPIDPublisher->publish(SwitcherPositionPIDMsg);
    }

    bool CoverController::reset_motor() {
        auto reset_request = std::make_shared<gary_msgs::srv::ResetMotorPosition::Request>();
        reset_request->motor_name = "cover";

        using namespace std::chrono_literals;
        while (!ResetMotorPositionClient->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service \'ResetMotorPositionClient\' not available, waiting again...");
        }

        using ServiceResponseFuture =
                rclcpp::Client<gary_msgs::srv::ResetMotorPosition>::SharedFuture;
        bool success = false;
        auto response_received_callback = [this,&success](ServiceResponseFuture future) {
            while (future.wait_for(2s) != std::future_status::ready){
                RCLCPP_DEBUG(this->get_logger(), "Waiting for Response....");
            }
            RCLCPP_DEBUG(this->get_logger(), "Received Response.");
            success = future.get()->succ;
        };

        auto reset_pos_result = ResetMotorPositionClient->async_send_request(reset_request,response_received_callback);
        if(!success){
            RCLCPP_ERROR(this->get_logger(), "Failed to reset position. Exiting.");
            return false;
        }
        return true;
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