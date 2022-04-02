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
#ifndef _ARDAK_JOINT_STATE_PUBLISHER_H_
#define _ARDAK_JOINT_STATE_PUBLISHER_H_

/**
 * ROS2 BASE INCLUDES
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/duration.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"

namespace zyg
{
    class JointStatePublisherNode : public rclcpp::Node
    {
    public:
        explicit JointStatePublisherNode(const rclcpp::NodeOptions &options);

    private:
        void publishJointState();
        void loop();

        sensor_msgs::msg::JointState front_right_wheel_state;
        sensor_msgs::msg::JointState front_left_wheel_state;
        sensor_msgs::msg::JointState rear_right_wheel_state;
        sensor_msgs::msg::JointState rear_left_wheel_state;

        // Only the two left drive motors have encoders on them.
        // Alignment for the front wheels is assumed.
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_left_wheel_encoder_subscription;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_right_wheel_encoder_subscription;

        rclcpp::Publisher<sensor_msgs::msg::JointState> front_right_wheel_state_publisher;
        rclcpp::Publisher<sensor_msgs::msg::JointState> front_left_wheel_state_publisher;
        rclcpp::Publisher<sensor_msgs::msg::JointState> rear_right_wheel_state_publisher;
        rclcpp::Publisher<sensor_msgs::msg::JointState> front_left_wheel_state_publisher;

        rclcpp::TimerBase::SharedPtr loopTimer;
    };
}

#endif // !_ARDAK_JOINT_STATE_PUBLISHER_H_