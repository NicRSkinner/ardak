/**
 * @file ardak_node.hpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Primary application control for the ardak system. Dictates incoming signals to determine
 *          functional state of the machine and signal downstream packages to start/stop operations.
 *        Handles switching from autonomous modes to manual mode, as well as any debugging implementations that
 *          need to signal downstream packages.
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _ARDAK_NODE_H_
#define _ARDAK_NODE_H_

/**
 * ROS2 BASE INCLUDES
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/duration.hpp"

/**
 * ROS2 MSGS INCLUDES
 */
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "bfr_msgs/msg/gamepad.hpp"

/**
 * ARDAK INCLUDES
 */
#include "gamepad_definitions.hpp"

namespace bfr
{
    class ArdakNode : public rclcpp::Node
    {
    public:
        explicit ArdakNode(const rclcpp::NodeOptions &options);

    private:
        void loop();

        bool systemEnabled = false;
        rclcpp::TimerBase::SharedPtr loopTimer;
        rclcpp::QoS base_qos = rclcpp::QoS(1);

        /**
         * @brief CALLBACKS
         */
        void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg);

        /**
         * @brief ROS2 TOPICS
         */
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr runPublisher;
        rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
    };
} // namespace bfr

#endif // !_ARDAK_NODE_H_
