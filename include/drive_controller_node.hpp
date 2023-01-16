/**
 * @file drive_controller_node.hpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Drive controller for Ardak project. Commands the steering and drive system
 *          as specificied by incoming signals from either a manual control or the
 *          onboard intelligence system.
 * @version 0.1
 * @date 2021-09-20
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _DRIVE_CONTROLLER_NODE_H_
#define _DRIVE_CONTROLLER_NODE_H_

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/**
 * ROS2 BASE INCLUDES
 */
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/duration.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

/**
 * ROS2 MSGS INCLUDES
 */
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "bfr_msgs/msg/gamepad.hpp"

/**
 * ARDAK INCLUDES
 */
#include "unittypes.h"
#include "gamepad_definitions.hpp"
#include "scale.hpp"

using namespace bfr_base::literals;

namespace bfr
{
    class DriveControllerNode : public rclcpp::Node
    {
    public:
        explicit DriveControllerNode(const rclcpp::NodeOptions &options);

    private:
        void loop();

        void input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo &event);
        void input_deadline_changed(rclcpp::QOSDeadlineRequestedInfo &event);
        void safety_liveliness_changed(rclcpp::QOSLivelinessChangedInfo &event);
        void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg);
        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        geometry_msgs::msg::Twist tank_to_twist(double leftVelocity, double rightVelocity);
        std::vector<double> twist_to_tank(geometry_msgs::msg::Twist input);

        bool inputAlive = false;
        bfr_msgs::msg::Gamepad lastReceivedMessage;
        rclcpp::QoS base_qos = rclcpp::QoS(1);

        /**
         * @brief PARAMETERS
         */
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callbackHandle;
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);
        bool manualControlAllowed = false;
        float driveGearRatio = 0.;
        double wheelbase = 0.f;
        bfr_base::Speed maxVelocity = bfr_base::Speed{0};
        bfr_base::Speed minVelocity = bfr_base::Speed{0};
        bfr_base::Length<std::centi> wheelCircumference = bfr_base::Length<std::centi>(0_cm);
        bfr_base::Speed maxSteeringVelocity = bfr_base::Speed{0};
        bfr_base::Speed minSteeringVelocity = bfr_base::Speed{0};
        bool running = false;

        double lastLeftCommand = 0.f;
        double lastRightCommand = 0.f;

        /**
         * @brief ROS2 TOPICS
         */
        rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdSubscription;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftDrivePublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightDrivePublisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistPublisher;

        /**
         * @brief ROS2 TIMERS
         */
        rclcpp::TimerBase::SharedPtr loopTimer;
    };
} // namespace bfr

#endif // !_DRIVE_CONTROLLER_NODE_H_
