#ifndef _DRIVE_CONTROLLER_NODE_H_
#define _DRIVE_CONTROLLER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/string.hpp"
#include "bfr_msgs/msg/gamepad.hpp"

namespace bfr
{
    class DriveControllerNode : public rclcpp::Node
    {
    public:
        explicit DriveControllerNode(const rclcpp::NodeOptions &options);

    private:
        void loop();
        
        void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg) const;
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

        bool gamepadEquipped = false;

        bfr_msgs::msg::Gamepad lastReceivedMessage;
        rclcpp::TimerBase::SharedPtr loopTimer;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callbackHandle;
        rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
    };
} // namespace bfr

#endif // !_DRIVE_CONTROLLER_NODE_H_
