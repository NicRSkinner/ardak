#ifndef _INPUT_CALIBRATION_NODE_
#define _INPUT_CALIBRATION_NODE_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
namespace bfr
{
    class InputCalibrationNode : public rclcpp::Node
    {
    public:
        explicit InputCalibrationNode(const rclcpp::NodeOptions &options);

        int getSteeringEncoderPosition();
        int getDriveEncoderPosition();

    private:
        void drive_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg);
        void steering_encoder_callback(const std_msgs::msg::Int32::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_encoder_subscription;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_encoder_subscription;
        
        int32_t steeringEncoderPosition;
        int32_t driveEncoderPosition;
    };
} // namespace bfr

#endif // !_INPUT_CALIBRATION_NODE_
