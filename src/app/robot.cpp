/**
 * @file Robot.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */

#include <rclcpp/rclcpp.hpp>
#include <iostream>

#include "app/Logger.hpp"
#include "app/Robotarm.hpp"
#include "dll/SSC32.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto robot = std::make_shared<Robotarm>(std::make_unique<SSC32>("/dev/ttyUSB0", 9600, 8));
    auto logger = std::make_shared<Logger>();

    executor.add_node(robot);
    executor.add_node(logger);
    executor.spin();

    rclcpp::shutdown();
}
