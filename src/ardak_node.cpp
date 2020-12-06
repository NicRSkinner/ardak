#include "ardak_node.h"

#include <chrono>

using namespace std::literals::chrono_literals;

namespace bfr
{
    ArdakNode::ArdakNode(const rclcpp::NodeOptions &options) : Node("ardak", options)
    {
        timer = this->create_wall_timer(10ms, std::bind(&ArdakNode::loop, this));
    }

    void ArdakNode::loop()
    {

    }
}