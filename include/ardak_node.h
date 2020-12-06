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

#endif // !_ARDAK_NODE_H_
