#include "ardak_node.h"

#include <chrono>
#include <iostream>

using namespace std::literals::chrono_literals;

namespace bfr
{
    ArdakNode::ArdakNode(const rclcpp::NodeOptions &options) : Node("ardak", options)
    {
        this->timer = this->create_wall_timer(50ms, std::bind(&ArdakNode::loop, this));
    }

    void ArdakNode::loop()
    {
    }
} // namespace bfr

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bfr::ArdakNode)