#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <string>
#include <cmath>
#include <chrono>

namespace gary_shoot{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HeatController : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit HeatController(const rclcpp::NodeOptions & options);


    private:

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    };

    HeatController::HeatController(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode(
            "heat_controller", options) {
        
    }

    CallbackReturn HeatController::on_configure(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_activate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HeatController::on_error(const rclcpp_lifecycle::State &previous_state) {
        RCL_UNUSED(previous_state);

        return CallbackReturn::SUCCESS;
    }


}


int main(int argc, char const *const argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<gary_shoot::HeatController> heatController =
            std::make_shared<gary_shoot::HeatController>(rclcpp::NodeOptions());

    exe.add_node(heatController->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_shoot::HeatController)