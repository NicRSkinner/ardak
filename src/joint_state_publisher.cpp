/**
 * @file ardak_node.cpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Handled publishing joint states for Ardak base robot.
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "joint_state_publisher.hpp"

namespace zyg
{
    JointStatePublisherNode::JointStatePublisherNode(const rclcpp::NodeOptions &options) : Node("ArdakJointStatePublisher", options)
    {
    }
}