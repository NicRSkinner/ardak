#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "input_calibration_node.h"
#include "ardak_node.h"

using namespace std;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

    auto input_cal_node = std::make_shared<bfr::InputCalibrationNode>(options);
    auto ardak_node = std::make_shared<bfr::ArdakNode>(options);
    exec.add_node(input_cal_node);
    exec.add_node(ardak_node);

    exec.spin();

    rclcpp::shutdown();

    return 0;
}