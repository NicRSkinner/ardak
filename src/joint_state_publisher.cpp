/**
 * @file joint_state_publisher.cpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Joint state publisher for base Ardak robot.
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "joint_state_publisher.hpp"
using std::placeholders::_1;

namespace zyg
{
    JointStatePublisherNode::JointStatePublisherNode(const rclcpp::NodeOptions &options) : Node("ArdakJointStatePublisher", options)
    {
        this->rear_left_wheel_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>("/odrive/axis0/encoder_counts", 10, 
            std::bind(&JointStatePublisherNode::leftEncoderCallback, this, _1));

        this->rear_right_wheel_encoder_subscription = this->create_subscription<std_msgs::msg::Int32>("/odrive/axis1/encoder_counts", 10, 
            std::bind(&JointStatePublisherNode::rightEncoderCallback, this, _1));

        this->joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    }

    void JointStatePublisherNode::leftEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->joined_left_wheel_position = msg->data;
        this->publishJointStates();
    }

    void JointStatePublisherNode::rightEncoderCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->joined_right_wheel_position = msg->data;
        this->publishJointStates();
    }

    /**
     * @brief Publishes all joint states for the robot.
     *          These states are taken in by the Robot State Publisher package
     *          to update the state for the entire robot.
     */
    void JointStatePublisherNode::publishJointStates()
    {
        sensor_msgs::msg::JointState robotJointStates;
        std::vector<std::string> jointNames = std::vector<std::string>();
        std::vector<double> jointPositions = std::vector<double>();
        
        jointNames.push_back("left_front_wheel");
        jointPositions.push_back(this->joined_left_wheel_position);

        jointNames.push_back("left_rear_wheel");
        jointPositions.push_back(this->joined_left_wheel_position);

        jointNames.push_back("right_front_wheel");
        jointPositions.push_back(this->joined_right_wheel_position);

        jointNames.push_back("right_rear_wheel");
        jointPositions.push_back(this->joined_right_wheel_position);

        robotJointStates.header.stamp = this->get_clock()->now();
        robotJointStates.name = jointNames;
        robotJointStates.position = jointPositions;
        
        this->joint_state_publisher->publish(robotJointStates);

        // https://docs.ros.org/en/foxy/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher.html#publish-the-state
    }

    void loop()
    {
        
    }
}