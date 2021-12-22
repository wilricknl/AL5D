/**
 * @file Logger.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "app/Logger.hpp"

Logger::Logger(): Node("Logger"){
    subscriptionInfo = this->create_subscription<std_msgs::msg::String>("info", 10, std::bind(&Logger::loginfo, this, std::placeholders::_1));
    subscriptionDebug = this->create_subscription<std_msgs::msg::String>("debug", 10, std::bind(&Logger::logdebug, this, std::placeholders::_1));
}

void Logger::loginfo(const std_msgs::msg::String::SharedPtr message) const {
    RCLCPP_INFO(this->get_logger(), "STATE: '%s'", message->data.c_str());
}

void Logger::logdebug(const std_msgs::msg::String::SharedPtr message) const {
    RCLCPP_DEBUG(this->get_logger(), "EVENT: '%s'", message->data.c_str());
}
