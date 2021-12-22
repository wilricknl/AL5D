/**
 * @file ServoCommand.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "ServoCommand.hpp"

ServoCommand::ServoCommand(char channel, unsigned short pulseWidth, unsigned int speed)
        : channel(channel), pulseWidth(pulseWidth), speed(speed)
{}

std::string ServoCommand::toString(unsigned short time) const
{
    std::string command {"#" + std::to_string(channel) + "P" + std::to_string(pulseWidth)};
    if(speed != 0){
        command += "S" + std::to_string(speed);
    }
    if(time != 0){
        command += "T" + std::to_string(time);
    }
    return command;
}

ServoCommands::ServoCommands(std::vector<ServoCommand> commands, unsigned short time)
: commands(commands), time(time)
{}

std::string ServoCommands::toString() const
{
    std::string string{};
    for (auto const& command : commands)
    {
        string += command.toString();
    }
    if(time != 0){
        string += "T" + std::to_string(time);
    }
    return string;
}

unsigned short ServoCommands::getTime() const {
    return time;
}
