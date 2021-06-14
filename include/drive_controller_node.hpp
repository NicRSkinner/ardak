#ifndef _DRIVE_CONTROLLER_NODE_H_
#define _DRIVE_CONTROLLER_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/duration.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int8.hpp"
#include "bfr_msgs/msg/gamepad.hpp"
#include "unittypes.h"
#include "gamepad_definitions.hpp"

namespace bfr
{
    class DriveControllerNode : public rclcpp::Node
    {
    public:
        explicit DriveControllerNode(const rclcpp::NodeOptions &options);

    private:
        void loop();
        
        void input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & event);
        void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg) const;
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);

        bool gamepadEquipped = false;

        bool inputAlive = false;
        uint8_t outputCommand = 0;

        bfr_base::Speed maxVelocity = bfr_base::Speed{0};
        bfr_base::Speed minVelocity = bfr_base::Speed{0};
        float driveGearRatio = 0.;

        bfr_msgs::msg::Gamepad lastReceivedMessage;
        rclcpp::TimerBase::SharedPtr loopTimer;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callbackHandle;
        rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr drivePublisher;
    };
} // namespace bfr

#endif // !_DRIVE_CONTROLLER_NODE_H_
