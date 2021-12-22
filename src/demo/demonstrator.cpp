/**
 * @file demonstrator.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "demo/Demo.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto demo = std::make_shared<demo::Demo>();
    while (not demo->isFinished())
    {
        demo->run();
    }
    rclcpp::shutdown();
}
