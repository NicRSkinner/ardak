#include "drive_controller_node.hpp"

#include <chrono>
#include <iostream>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace bfr
{
    DriveControllerNode::DriveControllerNode(const rclcpp::NodeOptions &options) : Node("DriveController", options)
    {
        this->callbackHandle = this->add_on_set_parameters_callback(
            std::bind(&DriveControllerNode::parametersCallback, this, std::placeholders::_1));

        this->declare_parameter("gamepadEquipped", false);
        this->get_parameter("gamepadEquipped", this->gamepadEquipped);


        this->drivePublisher = this->create_publisher<std_msgs::msg::Int8>(
            "appout/drive/drive_percent", 10
        );
    }

    rcl_interfaces::msg::SetParametersResult DriveControllerNode::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "Parameter not found/set.";

        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "gamepadEquipped" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                this->gamepadEquipped = parameter.as_bool();

                if (this->gamepadEquipped == false)
                {
                    // This should stop the callback from being served.
                    this->gamepadSubscription.reset();

                    this->loopTimer = this->create_wall_timer(50ms, std::bind(&DriveControllerNode::loop, this));
                }
                else
                {
                    this->loopTimer.reset();

                    this->gamepadSubscription = this->create_subscription<bfr_msgs::msg::Gamepad>(
                        "hal/inputs/gamepad", 10, std::bind(&DriveControllerNode::gamepad_callback, this, _1)
                    );
                }

                result.successful = true;
                result.reason = "NA";
            }
        }

        return result;
    }

    void DriveControllerNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg) const
    {
        if (msg->action == GamepadAction::LEFT_TRIGGER)
        {
            std_msgs::msg::Int8 message;
            message.data = msg->value;

            this->drivePublisher->publish(message);
        }
    }

    void DriveControllerNode::loop()
    {
        std::cout << "Test" << std::endl;
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.

// Ignore linting errors stating "int assumed" this seems to be a VSCode issue.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::DriveControllerNode)