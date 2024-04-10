/**
 * @file ardak_node.cpp
 * @author Nick Skinner (nicholas.skinner95@gmail.com)
 * @brief Primary application control for the ardak system. Dictates incoming
 * signals to determine functional state of the machine and signal downstream
 * packages to start/stop operations. Handles switching from autonomous modes to
 * manual mode, as well as any debugging implementations that need to signal
 * downstream packages.
 * @version 0.1
 *
 * @copyright Copyright (c) 2021-2024
 *
 */
#include "ardak_node.hpp"

/**
 * C++ BASE INCLUDES
 */
#include <chrono>
#include <exception>
#include <iostream>

#include "unitconversions.hpp"

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
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting ardak controller.");
        try
        {
            this->base_qos.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
            this->base_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            this->base_qos.liveliness_lease_duration(2s);
            this->base_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

            RCLCPP_INFO_STREAM(this->get_logger(), "Creating ROS objects.");
            this->runPublisher =
                this->create_publisher<std_msgs::msg::Bool>("safety/run", this->base_qos);
            this->gamepadSubscription = this->create_subscription<bfr_msgs::msg::Gamepad>(
                "hal/inputs/gamepad",
                this->base_qos,
                std::bind(&ArdakNode::gamepad_callback, this, _1)
            );

            this->gpsSubscription = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "gps/data",
                10,
                std::bind(&ArdakNode::gps_callback, this, _1)
            );

            this->headingSubscription = this->create_subscription<sensor_msgs::msg::MagneticField>(
                "mag/data",
                10,
                std::bind(&ArdakNode::heading_callback, this, _1)
            );

            this->loopTimer = this->create_wall_timer(150ms, std::bind(&ArdakNode::loop, this));
        }
        catch (std::exception &ex)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), ex.what());
        }
    }

    void ArdakNode::gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg)
    {
        if (msg->action == GamepadAction::ENABLE)
        {
            this->systemEnabled = true;
        }
    }

    void ArdakNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (msg.get()->status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            this->initialFix.altitude = msg.get()->altitude;
            this->initialFix.longitude = msg.get()->latitude;
            this->initialFix.longitude = msg.get()->longitude;
            this->initialFix.status = msg.get()->status;
        }
    }

    void ArdakNode::heading_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {}

    /**
     * @brief Basic timed loop for the ardak node. Handles timeouts and regularly
     * recurring items.
     *
     */
    void ArdakNode::loop()
    {
        std_msgs::msg::Bool msg;
        msg.data = true;

        this->runPublisher->publish(msg);
        // this->broadcastEarthToMapTF(); // For some reason this crashes the ardak
        // program
    }

    void ArdakNode::broadcastEarthToMapTF()
    {
        if (this->initialFix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        {
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
            geometry_msgs::msg::TransformStamped transformStamped;

            transformStamped.header.stamp = this->get_clock()->now();
            transformStamped.header.frame_id = "earth";
            transformStamped.child_frame_id = "map";
            transformStamped.transform.translation.x =
                zyg::LongitudeToMeters(0.f, this->initialFix.longitude);
            transformStamped.transform.translation.x =
                zyg::LatitudeToMeters(0.f, this->initialFix.latitude);
            transformStamped.transform.translation.z = 0.f;

            tf2::Quaternion rotationQuat;
            rotationQuat.setRPY(0, 0, 0);
            transformStamped.transform.rotation.x = rotationQuat.x();
            transformStamped.transform.rotation.y = rotationQuat.y();
            transformStamped.transform.rotation.z = rotationQuat.z();
            static_broadcaster->sendTransform(transformStamped);
        }
    }
} // namespace bfr
