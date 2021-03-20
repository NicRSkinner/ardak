#ifndef _INPUT_CALIBRATION_NODE_
#define _INPUT_CALIBRATION_NODE_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "unittypes.h"
#include "encoder.h"

using namespace bfr_base::literals;

namespace bfr
{
    class InputCalibrationNode : public rclcpp::Node
    {
    public:
        explicit InputCalibrationNode(const rclcpp::NodeOptions &options);

        int getSteeringEncoderPosition();
        int getDriveEncoderPosition();

    private:
        void drive_encoder_raw_callback(const std_msgs::msg::Int32::SharedPtr msg);
        void steering_encoder_raw_callback(const std_msgs::msg::Int32::SharedPtr msg);

        void loop();

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_encoder_raw_subscription;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_encoder_raw_subscription;
        
        int32_t steeringEncoderPosition = 0;
        int32_t driveEncoderPosition = 0;

        rclcpp::TimerBase::SharedPtr loopTimer;
        bfr_base::Encoder steeringEncoder;
        bfr_base::Encoder driveEncoder;
    };
} // namespace bfr

#endif // !_INPUT_CALIBRATION_NODE_
