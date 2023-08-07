#include "gary_shoot/barrel_switcher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <chrono>
#include <cmath>

namespace gary_shoot {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    BarrelSwitcher::BarrelSwitcher(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "barrel_switcher", options) {
        this->BarrelSwitchService = nullptr;
        SwitcherEffortPIDPublisher = nullptr;
//        this->ControllerSwitchClient = nullptr;
        this->timer_pub = nullptr;
        switched = false;
        switching = false;
        this->declare_parameter("effort_max", 0.3372);
        this->declare_parameter("effort_min", 0.2010);
        this->declare_parameter("switch_time_ms", 350.0);
        this->declare_parameter("delay_time_ms", 300.0);
        effort_max = 0.3372;
        effort_min = 0.2010;
        current_effort = effort_min;

        SWITCH_TIME = 400.0;
        DELAY_TIME = 450.0;
        current_pos = 0.0;
//        this->declare_parameter("tolerable_diff", M_PI_4 / 4);
//        this->tolerable_diff = M_PI_4 / 4;

//        this->declare_parameter("barrel_0_position", 0.0);
//        this->declare_parameter("barrel_1_position", 0.0);

//        barrel_id = 0.0;
//        current_position = 0.0;

//        this->need_zero = true;

    }

    CallbackReturn BarrelSwitcher::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

//        this->tolerable_diff =  this->get_parameter("tolerable_diff").as_double();
        this->effort_max =  this->get_parameter("effort_max").as_double();
        this->effort_min =  this->get_parameter("effort_min").as_double();
        this->SWITCH_TIME =  this->get_parameter("switch_time_ms").as_double();
        this->DELAY_TIME =  this->get_parameter("delay_time_ms").as_double();
        current_effort = effort_min;
//        this->position.emplace(0,this->get_parameter("barrel_0_position").as_double());
//        this->position.emplace(1,this->get_parameter("barrel_1_position").as_double());

        this->SwitchingPIDSubscription =
                this->create_subscription<gary_msgs::msg::PID>(
                        "/barrel_switch_position/pid",
                        rclcpp::SystemDefaultsQoS(),
                        std::bind(&BarrelSwitcher::pid_callback, this, std::placeholders::_1));

        SwitcherEffortPIDPublisher = create_publisher<std_msgs::msg::Float64>("/barrel_switch_pid_effort/cmd",rclcpp::SystemDefaultsQoS());
//        SwitcherPositionPIDPublisher = create_publisher<std_msgs::msg::Float64>("/barrel_switch_pid_position/cmd",rclcpp::SystemDefaultsQoS());

//        ControllerSwitchClient = create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
//        ResetMotorPositionClient = create_client<gary_msgs::srv::ResetMotorPosition>("/reset_motor_position");
        BarrelSwitchService = create_service<gary_msgs::srv::SwitchBarrel>
                ("/switch_barrel",std::bind(&BarrelSwitcher::switch_srv_callback,this,std::placeholders::_1,std::placeholders::_2));

//        this->need_zero = true;

        RCLCPP_INFO(this->get_logger(), "configured");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BarrelSwitcher::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(this->SwitcherEffortPIDPublisher.get()!= nullptr) SwitcherEffortPIDPublisher.reset();
        if(BarrelSwitchService.get() != nullptr) BarrelSwitchService.reset();
        if (SwitchingPIDSubscription.get() != nullptr) SwitchingPIDSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "cleaned up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BarrelSwitcher::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        using namespace std::chrono_literals;
        SwitcherEffortPIDMsg.data = 0;

        SwitcherEffortPIDPublisher->on_activate();

        this->timer_pub = this->create_wall_timer( 1000ms/100, [this] { publisher(); });

        RCLCPP_INFO(this->get_logger(), "activated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BarrelSwitcher::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(this->SwitcherEffortPIDPublisher.get()!= nullptr) SwitcherEffortPIDPublisher.reset();
        if(BarrelSwitchService.get() != nullptr) BarrelSwitchService.reset();
        if (SwitchingPIDSubscription.get() != nullptr) SwitchingPIDSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "deactivated");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BarrelSwitcher::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(this->SwitcherEffortPIDPublisher.get()!= nullptr) SwitcherEffortPIDPublisher.reset();
        if(BarrelSwitchService.get() != nullptr) BarrelSwitchService.reset();
        if (SwitchingPIDSubscription.get() != nullptr) SwitchingPIDSubscription.reset();

        RCLCPP_INFO(this->get_logger(), "shutdown");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BarrelSwitcher::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        if(this->SwitcherEffortPIDPublisher.get()!= nullptr) SwitcherEffortPIDPublisher.reset();
        if(BarrelSwitchService.get() != nullptr) BarrelSwitchService.reset();
        if (SwitchingPIDSubscription.get() != nullptr) SwitchingPIDSubscription.reset();

        RCLCPP_ERROR(this->get_logger(), "Error happened");
        return CallbackReturn::SUCCESS;
    }

    void BarrelSwitcher::switch_srv_callback(const std::shared_ptr<gary_msgs::srv::SwitchBarrel::Request> request,
                                             std::shared_ptr<gary_msgs::srv::SwitchBarrel::Response> response) {

        RCLCPP_DEBUG(this->get_logger(),"Received Call.");

        if(request->barrel_id == 0){
            if(switched){
                current_effort = effort_max;
                switching = true;
                RCLCPP_INFO(this->get_logger(),"Switching to barrel 0...");
            }
            switched = false;
        }else if(request->barrel_id == 1){
            if(!switched){
                current_effort = effort_max;
                switching = true;
                RCLCPP_INFO(this->get_logger(),"Switching to barrel 1...");
            }
            switched = true;
        }
        double threshold[2] = {-0.4,-2.8};
        response->success = !switching && (switched?(current_pos <= threshold[(int)switched]):(current_pos >= threshold[(int)switched]));
    }

    void BarrelSwitcher::publisher() {
	bool no_pub = false;
        static int state = 0;
        static double delay_time = DELAY_TIME;
        static double switch_time = SWITCH_TIME;
        if(switching){
            if(state == 0){
		no_pub = true;
                delay_time -= 10.0;
                if(delay_time <= 0.0){
                    state = 1;
                    delay_time = DELAY_TIME;
                    RCLCPP_INFO(this->get_logger(),"Pre-Delayed...");
                }
            }else if(state == 1){
                current_effort -= 10.0 * ((effort_max - effort_min) / SWITCH_TIME);
                switch_time -= 10.0;
                if(switch_time <= 0.0){
                    state = 2;
                    switch_time = SWITCH_TIME;
                    RCLCPP_INFO(this->get_logger(),"Switched!");
                }
            }else if (state == 2){
                delay_time -= 10.0;
                if(delay_time <= 0.0){
                    state = 0;
                    delay_time = DELAY_TIME;
                    switching = false;
                    RCLCPP_INFO(this->get_logger(),"Past-Delayed...");
                }
            }
        }
        if(!switched) {
            SwitcherEffortPIDMsg.data = 0+current_effort;
        }else{
            SwitcherEffortPIDMsg.data = 0-current_effort;
        }
	if(!no_pub){
            auto clock = rclcpp::Clock();
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), clock, 100, "setting: %lf", SwitcherEffortPIDMsg.data);
            if(SwitcherEffortPIDPublisher.get()!= nullptr) SwitcherEffortPIDPublisher->publish(SwitcherEffortPIDMsg);
	}
    }

    void BarrelSwitcher::pid_callback(gary_msgs::msg::PID::SharedPtr msg) {
        this->current_pos = msg->feedback;
    }

//    void BarrelSwitcher::setzero() {
//
//    }

//    void BarrelSwitcher::pid_callback(gary_msgs::msg::PID::SharedPtr msg) {
//        this->current_position = msg->feedback;
//    }

//    void BarrelSwitcher::setzero() {
//
//        auto controller_switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
//        controller_switch_request->stop_controllers.emplace_back("barrel_switch_pid_position");
//        controller_switch_request->start_controllers.emplace_back("barrel_switch_pid_effort");
//        controller_switch_request->start_asap = true;
//
//        using namespace std::chrono_literals;
//        while (!ControllerSwitchClient->wait_for_service(1s)) {
//            if (!rclcpp::ok()) {
//                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//                return;
//            }
//            RCLCPP_INFO(this->get_logger(), "Service \'SwitchController\' not available, waiting again...");
//        }
//
//        auto controller_switch_result = ControllerSwitchClient->async_send_request(controller_switch_request);
//        // Wait for the result.
//        while(controller_switch_result.wait_for(2s) != std::future_status::ready){
//            RCLCPP_INFO(this->get_logger(), "Waiting for \'SwitchController\' response...");
//            rclcpp::spin_some(this->get_node_base_interface());
//        }
//        if(!controller_switch_result.get()->ok){
//            RCLCPP_ERROR(this->get_logger(), "Failed to switch controller. Exiting.");
//            return;
//        }
//
//        SwitcherEffortPIDMsg.data = 0.5f;
//        auto timer = 3000ms;
//        while (rclcpp::ok()) {
//            if(SwitcherEffortPIDPublisher.get() != nullptr) {
//                SwitcherEffortPIDPublisher->publish(SwitcherEffortPIDMsg);
//                rclcpp::sleep_for(10ms);
//                timer -= 10ms;
//            }else{
//                break;
//            }
//        }
//
//        auto reset_request = std::make_shared<gary_msgs::srv::ResetMotorPosition::Request>();
//        reset_request->motor_name = "switch";
//
//        using namespace std::chrono_literals;
//        while (!ResetMotorPositionClient->wait_for_service(1s)) {
//            if (!rclcpp::ok()) {
//                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//                return;
//            }
//            RCLCPP_INFO(this->get_logger(), "Service \'ResetMotorPositionClient\' not available, waiting again...");
//        }
//        auto reset_pos_result = ResetMotorPositionClient->async_send_request(reset_request);
//        // Wait for the result.
//        while(reset_pos_result.wait_for(2s) != std::future_status::ready){
//            RCLCPP_INFO(this->get_logger(), "Waiting for \'ResetMotorPositionClient\' response...");
//            rclcpp::spin_some(this->get_node_base_interface());
//        }
//        if(!reset_pos_result.get()->succ){
//            RCLCPP_ERROR(this->get_logger(), "Failed to reset position. Exiting.");
//            return;
//        }
//    }


}


int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::BarrelSwitcher> barrelSwitcher =
            std::make_shared<gary_shoot::BarrelSwitcher>(rclcpp::NodeOptions());

    exe.add_node(barrelSwitcher->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::BarrelSwitcher)
