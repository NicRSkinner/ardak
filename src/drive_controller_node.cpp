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

        // TODO: Make all get parameters one call, for efficiency with the callback.
        this->declare_parameter<bool>("gamepadEquipped", false);
        this->declare_parameter<double>("maxVelocity", 0.0f);
        this->declare_parameter<double>("minVelocity", 0.0f);
        this->declare_parameter<double>("driveGearRatio", 0.0f);
        this->declare_parameter<double>("wheelCircumference", 0.0f);

        this->drivePublisher = this->create_publisher<std_msgs::msg::Float32>(
            "appout/drive/output_command", 10
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
            if (parameter.get_name() == "maxVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->maxVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "maxVelocity set";
            }

            if (parameter.get_name() == "minVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->minVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "minVelocity set";
            }

            if (parameter.get_name() == "driveGearRatio" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->driveGearRatio = parameter.as_double();

                result.successful = true;
                result.reason = "driveGearRatio set";
            }

            if (parameter.get_name() == "wheelCircumference" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->wheelCircumference.value = parameter.as_double();

                result.successful = true;
                result.reason = "wheelCircumference set";
            }

            if (parameter.get_name() == "gamepadEquipped" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                rclcpp::QoS input_qos(1);
                input_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
                input_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
                input_qos.liveliness_lease_duration(2s);
                input_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

                rclcpp::SubscriptionOptions sub_options;
                sub_options.event_callbacks.liveliness_callback = std::bind(&DriveControllerNode::input_liveliness_changed, this, _1);
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
                        "hal/inputs/gamepad", input_qos, std::bind(&DriveControllerNode::gamepad_callback, this, _1),
                        sub_options
                    );
                }

                result.successful = true;
                result.reason = "gamepadEquipped set";
            }
        }

        return result;
    }

    void DriveControllerNode::input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & event)
    {
        std::cout << "liveliness_changed! " << event.alive_count_change << std::endl;

        if (event.alive_count_change < 0)
        {
            this->inputAlive = false;
            this->outputVelocity.value = 0;
        }
        else if (event.alive_count_change > 0)
        {
            this->inputAlive = true;
        }
    }

    void DriveControllerNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg)
    {
        
        static bool rightPressedFirst;
        static bool leftPressedFrist;

        this->outputVelocity.value = bfr_base::scale(0., 100.,
            this->minVelocity.value, this->maxVelocity.value, static_cast<double>(msg->value));

        double outputTPS = (this->outputVelocity.value / 60.) * this->driveGearRatio;
        float outputTPSFloat = static_cast<float>(outputTPS);
        
        std_msgs::msg::Float32 output;

        if (this->inputAlive)
        {
            if (msg->action == GamepadAction::RIGHT_TRIGGER)
            {
                if (leftPressedFrist == false && this->outputVelocity.value != 0)
                {
                    rightPressedFirst = true;
                    output.data = outputTPSFloat;
                    this->drivePublisher->publish(output);
                }
                else if (rightPressedFirst == true && this->outputVelocity.value == 0)
                {
                    rightPressedFirst = false;
                    output.data = outputTPSFloat;
                    this->drivePublisher->publish(output);
                }                
            }
            else if (msg->action == GamepadAction::LEFT_TRIGGER)
            {
                if (rightPressedFirst == false && this->outputVelocity.value != 0)
                {
                    leftPressedFrist = true;
                    output.data = outputTPSFloat * -1;
                    this->drivePublisher->publish(output);
                }
                else if (leftPressedFrist == true && this->outputVelocity.value == 0)
                {
                    leftPressedFrist = false;
                    output.data = outputTPSFloat;
                    this->drivePublisher->publish(output);
                }
            }
            else if (msg->action == 127)
            {
                output.data = 0;
                this->drivePublisher->publish(output);
            }
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