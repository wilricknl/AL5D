/**
 * @file Demo.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "demo/Demo.hpp"

#include <algorithm>
#include <iostream>
#include <exception>

namespace demo
{
    const std::vector<std::string> commands { "init", "park", "ready", "straightup", "position", "stop", "quit", "exit" };

    Demo::Demo() : Node("DemoNode")
    {
        commandPublisher = this->create_publisher<std_msgs::msg::String>("command", 10);
        positionPublisher = this->create_publisher<robotarm::msg::Position>("custom", 10);
    }


    bool Demo::isFinished() const {
        return bFinished;
    }

    void Demo::run()
    {
        std::cout << "Enter a command: ";
        std::string line;
        if (std::getline(std::cin, line) and isValidCommand(line))
        {
            if (line == "quit" or line == "exit")
            {
                std::cout << "Exiting...\n";
                bFinished = true;
            }
            else if (line == "position")
            {
                positionCommand();
            }
            else
            {
                command(line);
            }
        }
        else
        {
            help();
        }
    }

    void Demo::command(std::string const& data) const {
        auto message{ std_msgs::msg::String() };
        message.data = data;
        commandPublisher->publish(message);
    }

    void Demo::positionCommand() const
    {
        int value{};
        auto message = robotarm::msg::Position();
        message.base = getInput("\tBase (degrees): ", value) ? value : 404;
        message.shoulder = getInput("\tShoulder (degrees): ", value) ? value : 404;
        message.elbow = getInput("\tElbow (degrees): ", value) ? value : 404;
        message.wrist = getInput("\tWrist (degrees): ", value) ? value : 404;
        message.gripper = getInput("\tGripper (degrees): ", value) ? value : 404;
        message.rotate = getInput("\tRotate (degrees): ", value) ? value : 404;
        message.time = getInput("\tTime (milliseconds): ", value) ? value : 1;
        positionPublisher->publish(message);
    }

    bool Demo::isValidCommand(const std::string &line) {
        return std::find(commands.begin(), commands.end(), line) != commands.end();
    }

    bool Demo::getInput(std::string const& message, int& outValue) const
    {
        std::cout << message;
        std::string input;
        if (std::getline(std::cin, input))
        {
            try
            {
                int tmp = std::stoi(input);
                outValue = tmp;
                return true;
            }
            catch (...)
            {
            }
        }
        return false;
    }

    void Demo::help() const {
        std::cout << "The following commands are allowed: \n\t[";
        std::copy(commands.begin(), commands.end() - 1,
                  std::ostream_iterator<std::string>(std::cout, ", "));
        std::cout << *(commands.end() - 1) << "]" << std::endl;
    }
} // namespace demo