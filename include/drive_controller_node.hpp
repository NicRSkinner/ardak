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
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
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
        
        void input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & event);
        void safety_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & event);
        void safety_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg);

        bool inputAlive = false;
        bool run = false;
        bfr_base::Speed outputVelocity = bfr_base::Speed{0};
        bfr_base::Degrees outputAngle = bfr_base::Degrees{0};
        bfr_msgs::msg::Gamepad lastReceivedMessage;
        rclcpp::QoS base_qos = rclcpp::QoS(1);

        /**
         * @brief PARAMETERS
         */
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callbackHandle;
        rcl_interfaces::msg::SetParametersResult parametersCallback(
            const std::vector<rclcpp::Parameter> &parameters);
        bool manualControlAllowed = false;
        bfr_base::Speed maxVelocity = bfr_base::Speed{0};
        bfr_base::Speed minVelocity = bfr_base::Speed{0};
        float driveGearRatio = 0.;
        float steeringGearRatio = 0.;
        bfr_base::Length<std::centi> wheelCircumference = bfr_base::Length<std::centi>(0_cm);
        bfr_base::Degrees maxSteeringAngle = bfr_base::Degrees{0};
        bfr_base::Degrees minSteeringAngle = bfr_base::Degrees{0};


        /**
         * @brief ROS2 TOPICS
         */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr runSubscription;
        rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drivePublisher;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steeringPublisher;

        /**
         * @brief ROS2 TIMERS
         */
        rclcpp::TimerBase::SharedPtr loopTimer;
    };
} // namespace bfr

#endif // !_DRIVE_CONTROLLER_NODE_H_
