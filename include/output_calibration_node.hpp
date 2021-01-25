#ifndef _OUTPUT_CALIBRATION_NODE_H_
#define _OUTPUT_CALIBRATION_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
namespace bfr
{
    class OutputCalibrationNode : public rclcpp::Node
    {
    public:
        explicit OutputCalibrationNode(const rclcpp::NodeOptions &options);
    };
} // namespace bfr

#endif // !_OUTPUT_CALIBRATION_NODE_H_
