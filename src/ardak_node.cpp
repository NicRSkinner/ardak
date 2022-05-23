/**
 * @file ardak_node.cpp
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
#include "ardak_node.hpp"

/**
 * C++ BASE INCLUDES
 */
#include <chrono>
#include <iostream>

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace bfr
{
    /**
     * @brief Construct a new Ardak Node:: Ardak Node object
     * 
     * @param options All node options that may be added by the launch script.
     */
    ArdakNode::ArdakNode(const rclcpp::NodeOptions &options) : Node("ardak", options)
    {
        this->base_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        this->base_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        this->base_qos.liveliness_lease_duration(2s);
        this->base_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        this->runPublisher = this->create_publisher<std_msgs::msg::Bool>("safety/run", this->base_qos);
        this->gamepadSubscription = this->create_subscription<bfr_msgs::msg::Gamepad>(
            "hal/inputs/gamepad", this->base_qos, std::bind(&ArdakNode::gamepad_callback, this, _1)
        );
        
        this->loopTimer = this->create_wall_timer(150ms, std::bind(&ArdakNode::loop, this));
    }

    void ArdakNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg)
    {
        if (msg->action == GamepadAction::ENABLE)
        {
            this->systemEnabled = true;
        }
    }

    /**
     * @brief Basic timed loop for the ardak node. Handles timeouts and regularly recurring items.
     * 
     */
    void ArdakNode::loop()
    {
        std_msgs::msg::Bool msg;
        msg.data = this->systemEnabled;

        this->runPublisher->publish(msg);
    }
} // namespace bfr