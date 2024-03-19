#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "drive_controller_node.hpp"
#include "ardak_node.hpp"

using namespace std;

int main(int argc, char* argv[])
{
    cout << "Initializing ROS" << endl;
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    cout << "Initializing ROS nodes" << endl;
    auto drive_control_node = std::make_shared<bfr::DriveControllerNode>(options);
    auto ardak_node = std::make_shared<bfr::ArdakNode>(options);

    cout << "Adding Drive Controller node" << endl;
    exec.add_node(drive_control_node);

    cout << "Adding Ardak main node" << endl;
    exec.add_node(ardak_node);

    cout << "Performing ROS node tasks..." << endl;
    exec.spin();

    cout << "Shutting down ROS environment" << endl;
    rclcpp::shutdown();

    return 0;
}