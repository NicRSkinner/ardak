#include "input_calibration_node.h"
#include <iostream>

using std::placeholders::_1;
using namespace std::literals::chrono_literals;

namespace bfr
{
    InputCalibrationNode::InputCalibrationNode(const rclcpp::NodeOptions &options) : Node("InputCalibration", options)
    {
        this->steeringEncoder = bfr_base::Encoder(1024, 1, 1_cm);
        this->driveEncoder = bfr_base::Encoder(1024, 1, 79.756_cm);

        this->steering_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/steering_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));

        this->drive_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>(
            "hardware/inputs/drive_encoder", 10, std::bind(&InputCalibrationNode::drive_encoder_callback, this, _1));

        this->loopTimer = this->create_wall_timer(50ms, std::bind(&InputCalibrationNode::loop, this));
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
        static int32_t prevDriveEncoderPosition = 0;
        static int32_t prevSteeringEncoderPosition = 0;

        // Tire is a 10" wheel with a 1024 pulse per rev encoder.
        this->driveEncoder.update(this->driveEncoderPosition - prevDriveEncoderPosition, 50ms);

        prevDriveEncoderPosition = this->driveEncoderPosition;

        std::cout << this->driveEncoder.getSpeed().value << "m/hr " << this->driveEncoder.getPosition().value << "cm" << std::endl;
    }
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::InputCalibrationNode)