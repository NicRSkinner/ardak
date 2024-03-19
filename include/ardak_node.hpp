/**
 * @file ardak_node.hpp
 * @author Nick Skinner (nskinner@zygenrobotics.com)
 * @brief Primary application control for the ardak system. Dictates incoming
 * signals to determine functional state of the machine and signal downstream
 * packages to start/stop operations. Handles switching from autonomous modes to
 * manual mode, as well as any debugging implementations that need to signal
 * downstream packages.
 * @version 0.1
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _ARDAK_NODE_H_
#define _ARDAK_NODE_H_

/**
 * ROS2 BASE INCLUDES
 */
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * TF2 INCLUDES
 */
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * ROS2 MSGS INCLUDES
 */
#include "bfr_msgs/msg/gamepad.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * ARDAK INCLUDES
 */
#include "gamepad_definitions.hpp"

namespace bfr
{
  class ArdakNode : public rclcpp::Node
  {
  public:
    explicit ArdakNode(const rclcpp::NodeOptions &options);

  private:
    void loop();
    void broadcastEarthToMapTF();

    bool systemEnabled = false;
    bool gpsLocked = false;
    rclcpp::TimerBase::SharedPtr loopTimer;
    rclcpp::QoS base_qos = rclcpp::QoS(1);
    sensor_msgs::msg::NavSatFix initialFix;

    /**
     * @brief CALLBACKS
     */
    void gamepad_callback(const bfr_msgs::msg::Gamepad::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void heading_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

    /**
     * @brief ROS2 TOPICS
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr runPublisher;
    rclcpp::Subscription<bfr_msgs::msg::Gamepad>::SharedPtr gamepadSubscription;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
        gpsSubscription;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
        headingSubscription;
  };
} // namespace bfr

#endif // !_ARDAK_NODE_H_
