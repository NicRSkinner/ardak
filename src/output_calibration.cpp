#include "output_calibration_node.hpp"

using std::placeholders::_1;

namespace bfr
{
    OutputCalibrationNode::OutputCalibrationNode(const rclcpp::NodeOptions &options) : Node("OutputCalibration", options)
    {
    }
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::OutputCalibrationNode)