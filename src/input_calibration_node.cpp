#include "input_calibration_node.h"

using std::placeholders::_1;
using namespace std::literals::chrono_literals;

#include <iostream>

namespace bfr
{
    InputCalibrationNode::InputCalibrationNode(const rclcpp::NodeOptions &options) : Node("InputCalibration", options)
    {
        this->steering_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/steering_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));

        this->drive_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/drive_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));

        this->timer = this->create_wall_timer(50ms, std::bind(&InputCalibrationNode::loop, this));
    }

    void InputCalibrationNode::drive_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->steeringEncoderPosition = msg->data;
    }

    void InputCalibrationNode::steering_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->driveEncoderPosition = msg->data;
    }

    void InputCalibrationNode::loop()
    {
    }
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::InputCalibrationNode)