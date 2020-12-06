#ifndef _ARDAK_NODE_H_
#define _ARDAK_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace bfr
{
    class ArdakNode : public rclcpp::Node
    {
    public:
        explicit ArdakNode(const rclcpp::NodeOptions &options);

    private:
        void loop();

        rclcpp::TimerBase::SharedPtr timer;
    };
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::ArdakNode)

#endif // !_ARDAK_NODE_H_
