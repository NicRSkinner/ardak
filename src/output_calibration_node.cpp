#include "output_calibration_node.hpp"

using std::placeholders::_1;

namespace bfr
{
    OutputCalibrationNode::OutputCalibrationNode(const rclcpp::NodeOptions &options) : Node("OutputCalibration", options)
    {
        this->driveSubscription = this->create_subscription<std_msgs::msg::Int8>(
            "hal/inputs/gamepad", 10, std::bind(&OutputCalibrationNode::drive_callback, this, _1)
        );

        this->drivePublisher = this->create_publisher<std_msgs::msg::Int8>(
            "hal/outputs/pwm1_duty", 10
        );
    }

    void OutputCalibrationNode::drive_callback(const std_msgs::msg::Int8::SharedPtr msg) const
    {
        std_msgs::msg::Int8 output;
        output.data = msg->data;

        this->drivePublisher->publish(output);
    }
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::OutputCalibrationNode)