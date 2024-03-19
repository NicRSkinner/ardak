#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "joint_state_publisher.hpp"
#include "robot_state_publisher/robot_state_publisher.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    cout << "Initializing ROS" << endl;
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    cout << "Initializing ROS nodes" << endl;
    auto joint_state_node = std::make_shared<zyg::JointStatePublisherNode>(options);
    auto robot_state_node = std::make_shared<robot_state_publisher::RobotStatePublisher>(options);

    cout << "Adding joint state publisher node" << endl;
    exec.add_node(joint_state_node);

    cout << "Adding robot state publisher node" << endl;
    exec.add_node(robot_state_node);

    cout << "Performing ROS node tasks..." << endl;
    exec.spin();

    cout << "Shutting down ROS environment" << endl;
    rclcpp::shutdown();

    return 0;
}