/**
 * @file joint_state_publisher.hpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Joint state publisher for base Ardak robot.
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
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

#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"

#include <vector>

namespace zyg
{
    class JointStatePublisherNode : public rclcpp::Node
    {
    public:
        explicit JointStatePublisherNode(const rclcpp::NodeOptions &options);

    private:
        void leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void publishJointStates();
        void loop();

        sensor_msgs::msg::JointState front_right_wheel_state;
        sensor_msgs::msg::JointState front_left_wheel_state;
        sensor_msgs::msg::JointState rear_right_wheel_state;
        sensor_msgs::msg::JointState rear_left_wheel_state;

        // Only the two left drive motors have encoders on them.
        // Alignment for the front wheels is assumed.
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_left_wheel_encoder_subscription;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rear_right_wheel_encoder_subscription;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;

        rclcpp::TimerBase::SharedPtr loopTimer;

        int joined_right_wheel_position = 0;
        int joined_left_wheel_position = 0;
    };
}

#endif // !_ARDAK_JOINT_STATE_PUBLISHER_H_