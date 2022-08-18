/**
 * @file drive_controller_node.cpp
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

/**
 * C++ BASE INCLUDES
 */
#include <chrono>
#include <iostream>


#include "drive_controller_node.hpp"

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace bfr
{
    DriveControllerNode::DriveControllerNode(const rclcpp::NodeOptions &options) : Node("DriveController", options)
    {
        this->base_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
        this->base_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        this->base_qos.liveliness_lease_duration(2s);
        this->base_qos.deadline(500ms);
        this->base_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        this->callbackHandle = this->add_on_set_parameters_callback(
            std::bind(&DriveControllerNode::parametersCallback, this, std::placeholders::_1));

        this->declare_parameter<bool>("manualControlAllowed", true);
        this->declare_parameter<double>("maxVelocity", 0.0f);
        this->declare_parameter<double>("minVelocity", 0.0f);
        this->declare_parameter<double>("driveGearRatio", 0.0f);
        this->declare_parameter<double>("wheelCircumference", 0.0f);
        this->declare_parameter<double>("maxSteeringVelocity", 0.0f);
        this->declare_parameter<double>("minSteeringVelocity", 0.0f);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.event_callbacks.liveliness_callback = std::bind(&DriveControllerNode::input_liveliness_changed, this, _1);
        sub_options.event_callbacks.deadline_callback = std::bind(&DriveControllerNode::input_deadline_changed, this, _1);

        this->leftDrivePublisher = this->create_publisher<std_msgs::msg::Float32>(
            "appout/drive/left_drive_command", 10
        );

        this->rightDrivePublisher = this->create_publisher<std_msgs::msg::Float32>(
            "appout/drive/right_drive_command", 10
        );
    }

    rcl_interfaces::msg::SetParametersResult DriveControllerNode::parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "Parameter not found/set.";

        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "maxVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->maxVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "maxVelocity set";
            }

            if (parameter.get_name() == "minVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->minVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "minVelocity set";
            }

            if (parameter.get_name() == "driveGearRatio" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->driveGearRatio = parameter.as_double();

                result.successful = true;
                result.reason = "driveGearRatio set";
            }

            if (parameter.get_name() == "wheelCircumference" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->wheelCircumference.value = parameter.as_double();

                result.successful = true;
                result.reason = "wheelCircumference set";
            }

            if (parameter.get_name() == "maxSteeringVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->maxSteeringVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "maxSteeringVelocity set";
            }

            if (parameter.get_name() == "minSteeringVelocity" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->minSteeringVelocity.value = parameter.as_double();

                result.successful = true;
                result.reason = "minSteeringVelocity set";
            }

            if (parameter.get_name() == "manualControlAllowed" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                rclcpp::SubscriptionOptions sub_options;
                sub_options.event_callbacks.liveliness_callback = std::bind(&DriveControllerNode::input_liveliness_changed, this, _1);
                sub_options.event_callbacks.deadline_callback = std::bind(&DriveControllerNode::input_deadline_changed, this, _1);
                this->manualControlAllowed = parameter.as_bool();

                if (this->manualControlAllowed == false)
                {
                    // This should stop the callback from being served.
                    this->gamepadSubscription.reset();

                    this->loopTimer = this->create_wall_timer(10ms, std::bind(&DriveControllerNode::loop, this));
                }
                else
                {
                    this->loopTimer.reset();

                    this->gamepadSubscription = this->create_subscription<bfr_msgs::msg::Gamepad>(
                        "hal/inputs/gamepad", this->base_qos, std::bind(&DriveControllerNode::gamepad_callback, this, _1),
                        sub_options
                    );
                }

                result.successful = true;
                result.reason = "gamepadEquipped set";
            }
        }

        return result;
    }

    void DriveControllerNode::input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo & event)
    {
        std::cout << "Input liveliness changed! " << event.alive_count_change << std::endl;
        if (event.alive_count_change < 0)
        {
            this->inputAlive = false;
        }
        else if (event.alive_count_change > 0)
        {
            this->inputAlive = true;
        }
    }

    void DriveControllerNode::input_deadline_changed(rclcpp::QOSDeadlineRequestedInfo & event)
    {
        // We've missed a message.
        if (event.total_count_change > 0)
        {
            //std_msgs::msg::Float32 output;
            //output.data = 0.f;
            //this->leftDrivePublisher->publish(output);
            //this->rightDrivePublisher->publish(output);
        }
    }

    void DriveControllerNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg)
    {
        if (msg->action == GamepadAction::ENABLE)
        {
                this->running = true;
                RCLCPP_INFO(this->get_logger(), std::string("DriveController: Manual control enabled.").c_str());
        }

        if (this->inputAlive && this->running)
        {
            if (msg->action == GamepadAction::RIGHT_STICK_UP_DOWN)
            {
                double outputVelocity = 0.f;
                double outputRPH = 0.f;
                double outputRPM  = 0.f;
                double outputTPS = 0.f;
                std_msgs::msg::Float32 output;
                output.data = 0.f;

                if (msg->value > 10 || msg->value < -10)
                {
                    outputVelocity = bfr_base::scale(-100., 100.,
                        this->minVelocity.value, this->maxVelocity.value, static_cast<double>(msg->value));
                    
                    outputRPH = (outputVelocity * 100.) / this->wheelCircumference.value;
                    outputRPM = outputRPH / 60.;
                    outputTPS = outputRPM / 60.;                   
                    
                    output.data = static_cast<float>(outputTPS / this->driveGearRatio);
                    // This should cap out around 18TPS
                }
                
                this->rightDrivePublisher->publish(output);
            }
            else if (msg->action == GamepadAction::LEFT_STICK_UP_DOWN)
            {
                double outputVelocity = 0.f;
                double outputRPH = 0.f;
                double outputRPM  = 0.f;
                double outputTPS = 0.f;
                std_msgs::msg::Float32 output;
                output.data = 0.f;

                if (msg->value > 10 || msg->value < -10)
                {
                    outputVelocity = bfr_base::scale(-100., 100.,
                        this->minVelocity.value, this->maxVelocity.value, static_cast<double>(msg->value));
                    
                    outputRPH = (outputVelocity * 100.) / this->wheelCircumference.value;
                    outputRPM = outputRPH / 60.;
                    outputTPS = outputRPM / 60.;                   
                    
                    output.data = static_cast<float>(outputTPS / this->driveGearRatio);
                    // This should cap out around 18TPS
                }
                
                this->leftDrivePublisher->publish(output);
            }
        }
    }

    void DriveControllerNode::loop()
    {
        std::cout << "Test" << std::endl;
    }
}