#include "input_calibration_node.h"

using std::placeholders::_1;

namespace bfr
{
    InputCalibrationNode::InputCalibrationNode(const rclcpp::NodeOptions &options) : Node("InputCalibration", options)
    {
        this->steering_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/steering_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));

        this->drive_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/drive_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));
    }

    void InputCalibrationNode::drive_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->steeringEncoderPosition = msg->data;
    }

    void InputCalibrationNode::steering_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->driveEncoderPosition = msg->data;
    }
} // namespace bfr