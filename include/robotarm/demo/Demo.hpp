/**
 * @file Demo.hpp
 * @brief Demo is replacement for the Motion Planner for testing
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_DEMO_HPP
#define ROBOTARM_DEMO_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robotarm/msg/position.hpp"

#include <string>
#include <vector>

namespace demo
{
    /**
     * @brief Demo is replacement for the Motion Planner for testing
     */
    class Demo : public rclcpp::Node
    {
    public:
        Demo();

        /**
         * @brief Run the demo application
         */
        void run();

        /**
         * @brief Check if Demo is finished
         * @return `true` if Demo is finished, else `false`
         */
        bool isFinished() const;
    private:
        /**
         * @brief Publish a command
         *
         * @param data The command
         */
        void command(std::string const& data) const;

        /**
         * @brief Publish a custom position
         */
        void positionCommand() const;

        /**
         * @brief Check if the entered command is valid
         *
         * @param line The command
         *
         * @return `true` if command is valid, else `false`
         */
        bool isValidCommand(std::string const& line);

        /**
         * @brief Get input from the user
         *
         * @param message Description of expected input
         * @param outValue The entered value
         *
         * @return `true` if entered input is valid, else `false`
         */
        bool getInput(std::string const& message, int& outValue) const;

        /**
         * @brief Print help message about valid commands
         */
        void help() const;
    private:
        bool bFinished = false; ///< Demo status
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr commandPublisher; ///< Command publisher
        rclcpp::Publisher<robotarm::msg::Position>::SharedPtr positionPublisher; ///< Custom position publisher
    };
} // namespace demo

#endif //ROBOTARM_DEMO_HPP
