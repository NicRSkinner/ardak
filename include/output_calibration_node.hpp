#ifndef _OUTPUT_CALIBRATION_NODE_H_
#define _OUTPUT_CALIBRATION_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float32.hpp"

namespace bfr
{
    class OutputCalibrationNode : public rclcpp::Node
    {
    public:
        explicit OutputCalibrationNode(const rclcpp::NodeOptions &options);

    private:
        void drive_callback(const std_msgs::msg::Int8::SharedPtr msg) const;

        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr driveSubscription;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drivePublisher;
    };
} // namespace bfr

#endif // !_OUTPUT_CALIBRATION_NODE_H_
