/**
 * @file Logger.hpp
 * @brief Logger performs all ROS2 logging
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_LOGGER_H
#define ROBOTARM_LOGGER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief The log levels
 */
enum class Logtype
{
    INFO,
    DEBUG
};

/**
 * @brief Logger performs all ROS2 logging
 */
class Logger : public rclcpp::Node
{
public:
    Logger();
private:
    /**
     * @brief The info logger callback
     *
     * @param message The message to be logged
     */
    void loginfo(const std_msgs::msg::String::SharedPtr message) const;

    /**
     * @brief The debug logger callback
     *
     * @param message The message to be logged
     */
    void logdebug(const std_msgs::msg::String::SharedPtr message) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionInfo; ///< Info log subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriptionDebug; ///< Debug log subscriber
};

#endif //ROBOTARM_LOGGER_H
