/**
 * @file drive_controller_node.cpp
 * @author Nick Skinner (nicholas.skinner95@gmail.com)
 * @brief Drive controller for Ardak project. Commands the steering and drive
 * system as specificied by incoming signals from either a manual control or the
 *          onboard intelligence system.
 * @version 0.2
 * @date 2024-04-10
 *
 * @copyright Copyright (c) 2021-2024
 *
 */

/**
 * C++ BASE INCLUDES
 */
#include <chrono>
#include <exception>
#include <iostream>
#include <math.h>

#include "drive_controller_node.hpp"

using namespace std::literals::chrono_literals;
using std::placeholders::_1;

namespace bfr
{
    DriveControllerNode::DriveControllerNode(const rclcpp::NodeOptions &options)
        : Node("DriveController", options)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting drive controller.");

        try
        {
            //this->base_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
            //this->base_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            this->base_qos.liveliness_lease_duration(2s);
            this->base_qos.deadline(500ms);
            //this->base_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

            this->callbackHandle = this->add_on_set_parameters_callback(
                std::bind(&DriveControllerNode::parametersCallback, this, std::placeholders::_1)
            );

            RCLCPP_INFO_STREAM(this->get_logger(), "Declaring parameters.");
            this->declare_parameter<bool>("manualControlAllowed", true);
            this->declare_parameter<double>("maxVelocity", 0.0f);
            this->declare_parameter<double>("minVelocity", 0.0f);
            this->declare_parameter<double>("driveGearRatio", 0.0f);
            this->declare_parameter<double>("wheelCircumference", 0.0f);
            this->declare_parameter<double>("maxSteeringVelocity", 0.0f);
            this->declare_parameter<double>("wheelbase", 0.0f);

            RCLCPP_INFO_STREAM(this->get_logger(), "Creating Subscription.");
            rclcpp::SubscriptionOptions sub_options;
            sub_options.event_callbacks.liveliness_callback =
                std::bind(&DriveControllerNode::input_liveliness_changed, this, _1);
            sub_options.event_callbacks.deadline_callback =
                std::bind(&DriveControllerNode::input_deadline_changed, this, _1);

            RCLCPP_INFO_STREAM(this->get_logger(), "Creating publishers.");
            this->leftDrivePublisher = this->create_publisher<std_msgs::msg::Float32>(
                "appout/drive/left_drive_command",
                10
            );

            this->rightDrivePublisher = this->create_publisher<std_msgs::msg::Float32>(
                "appout/drive/right_drive_command",
                10
            );

            // This echos the input for autonomous systems, and calculates for
            // tank-drive manual inputs
            this->twistPublisher =
                this->create_publisher<geometry_msgs::msg::Twist>("appout/drive/wheel_cmd", 10);
        }
        catch (std::exception &ex)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), ex.what());
            exit(-1);
        }
    }

    rcl_interfaces::msg::SetParametersResult
    DriveControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
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

            if (parameter.get_name() == "wheelbase" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                this->wheelbase = parameter.as_double();

                result.successful = true;
                result.reason = "wheelbase set";
            }

            if (parameter.get_name() == "manualControlAllowed" &&
                parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                rclcpp::SubscriptionOptions sub_options;
                sub_options.event_callbacks.liveliness_callback =
                    std::bind(&DriveControllerNode::input_liveliness_changed, this, _1);
                sub_options.event_callbacks.deadline_callback =
                    std::bind(&DriveControllerNode::input_deadline_changed, this, _1);
                this->manualControlAllowed = parameter.as_bool();

                if (this->manualControlAllowed == false)
                {
                    // This should stop the callback from being served.
                    this->gamepadSubscription.reset();

                    this->cmdSubscription = this->create_subscription<geometry_msgs::msg::Twist>(
                        "cmd_vel",
                        10,
                        std::bind(&DriveControllerNode::twist_callback, this, _1)
                    );

                    // this->loopTimer = this->create_wall_timer(10ms,
                    //  std::bind(&DriveControllerNode::loop, this));
                }
                else
                {
                    this->cmdSubscription.reset();

                    this->gamepadSubscription = this->create_subscription<bfr_msgs::msg::Gamepad>(
                        "hal/inputs/gamepad",
                        rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
                        std::bind(&DriveControllerNode::gamepad_callback, this, _1)
                    );
                        //sub_options
                }

                result.successful = true;
                result.reason = "gamepadEquipped set";
            }
        }

        return result;
    }

    void DriveControllerNode::input_liveliness_changed(rclcpp::QOSLivelinessChangedInfo &event)
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

    void DriveControllerNode::input_deadline_changed(rclcpp::QOSDeadlineRequestedInfo &event)
    {
        // RCLCPP_INFO(this->get_logger(), std::string("DriveController: Message
        // missed!").c_str());

        // We've missed a message.
        /*if (event.total_count_change > 0)
        {
            std_msgs::msg::Float32 output;
            output.data = 0.f;
            this->lastRightCommand = 0.f;
            this->lastLeftCommand = 0.f;
            this->leftDrivePublisher->publish(output);
            this->rightDrivePublisher->publish(output);
            this->twistPublisher->publish(this->tank_to_twist(this->lastLeftCommand,
        this->lastRightCommand));
        }*/
    }

    // Uses a dual-stick input from the gamepad for manual control
    void DriveControllerNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg)
    {
        if (msg->action == GamepadAction::ENABLE)
        {
            this->running = true;
            RCLCPP_INFO(
                this->get_logger(),
                std::string("DriveController: Manual control enabled.").c_str()
            );
        }

        this->running = true;

        if (this->running)
        {
            if (msg->action == GamepadAction::RIGHT_STICK_UP_DOWN)
            {
                double outputVelocity = 0.f;
                double outputRPH = 0.f;
                double outputRPM = 0.f;
                double outputTPS = 0.f;
                std_msgs::msg::Float32 output;
                output.data = 0.f;

                if (msg->value > 10 || msg->value < -10)
                {
                    outputVelocity = bfr_base::scale(
                        -100.,
                        100.,
                        this->minVelocity.value,
                        this->maxVelocity.value,
                        static_cast<double>(msg->value)
                    );

                    outputRPH = (outputVelocity * 100.) / this->wheelCircumference.value;
                    outputRPM = outputRPH / 60.;
                    outputTPS = outputRPM / 60.;

                    output.data = static_cast<float>(outputTPS / this->driveGearRatio);
                    // This should cap out around 18TPS
                }

                this->lastRightCommand = (outputRPM * 2 * 3.14f) / 60.0;
                this->rightDrivePublisher->publish(output);
            }
            else if (msg->action == GamepadAction::LEFT_STICK_UP_DOWN)
            {
                double outputVelocity = 0.f;
                double outputRPH = 0.f;
                double outputRPM = 0.f;
                double outputTPS = 0.f;
                std_msgs::msg::Float32 output;
                output.data = 0.f;

                if (msg->value > 10 || msg->value < -10)
                {
                    outputVelocity = bfr_base::scale(
                        -100.,
                        100.,
                        this->minVelocity.value,
                        this->maxVelocity.value,
                        static_cast<double>(msg->value)
                    );

                    outputRPH = (outputVelocity * 100.) / this->wheelCircumference.value;
                    outputRPM = outputRPH / 60.;
                    outputTPS = outputRPM / 60.;

                    output.data = static_cast<float>(outputTPS / this->driveGearRatio);
                    // This should cap out around 18TPS
                }

                this->lastLeftCommand = (outputRPM * 2 * 3.14f) / 60.0;
                this->leftDrivePublisher->publish(output);
            }

            this->twistPublisher->publish(
                this->tank_to_twist(this->lastLeftCommand, this->lastRightCommand)
            );
        }
    }

    void DriveControllerNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist out;
        out.angular = msg.get()->angular;
        out.linear = msg.get()->linear;

        // For simulation, re-output as a remapping. Tank drive for real-world
        // applications.
        this->twistPublisher->publish(out);

        this->twist_to_tank(out);
    }

    geometry_msgs::msg::Twist
    DriveControllerNode::tank_to_twist(double leftWheelVelocity, double rightWheelVelocity)
    {
        geometry_msgs::msg::Twist outputTwist;

        // Linear velocity = (Right wheel velocity + Left wheel velocity) / 2

        outputTwist.linear.x = (rightWheelVelocity + leftWheelVelocity) / 2;
        outputTwist.linear.y = 0.0f; // Always 0 (we cannot scoot sideways)
        outputTwist.linear.z = 0.0f; // Always 0 (we cannot fly)

        // Angular velocity = (Right wheel velocity - Left wheel velocity) /
        // wheelbase

        outputTwist.angular.x = 0.0f; // Always 0 (we cannot fly)
        outputTwist.angular.y = 0.0f; // Always 0 (we cannot fly)
        outputTwist.angular.z = (rightWheelVelocity - leftWheelVelocity) / this->wheelbase;

        return outputTwist;
    }

    void DriveControllerNode::twist_to_tank(geometry_msgs::msg::Twist input)
    {
        std_msgs::msg::Float32 leftMsg;
        std_msgs::msg::Float32 rightMsg;

        double RPMInRads = 0.1047;
        double DistPerRad = (this->wheelCircumference.value / 100) / (2 * M_PI);

        double left_vel = input.linear.x - input.angular.z * this->wheelbase / 2.0;
        double right_vel = input.linear.x + input.angular.z * this->wheelbase / 2.0;

        double left_rpm = left_vel / (RPMInRads * DistPerRad);
        double right_rpm = right_vel / (RPMInRads * DistPerRad);

        leftMsg.data = (left_rpm / 60. / this->driveGearRatio);
        rightMsg.data = (right_rpm / 60. / this->driveGearRatio);

        /*RCLCPP_INFO_STREAM(
            this->get_logger(),
            "linear: " << input.linear.x << " angular: " << input.angular.z
                       << "\n leftRPM: " << left_rpm << " rightRPM: " << right_rpm
                       << "\n leftTPS: " << leftMsg.data << " rightTPS: " << rightMsg.data
        );
        */

        this->leftDrivePublisher->publish(leftMsg);
        this->rightDrivePublisher->publish(rightMsg);
    }

    void DriveControllerNode::loop() { std::cout << "Test" << std::endl; }
} // namespace bfr
