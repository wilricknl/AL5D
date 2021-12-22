/**
 * @file ServoCommand.hpp
 * @brief Structures to easily pass and store servo command data
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_SERVOCOMMAND_HPP
#define ROBOTARM_SERVOCOMMAND_HPP

#include <string>
#include <vector>

/**
 * @brief ServoCommand stores a command for a servo
 */
class ServoCommand
{
public:
    /**
     * @brief ServoCommand constructor
     *
     * @param channel Servo channel
     * @param pulseWidth Pulsewidth
     * @param speed Speed
     */
    ServoCommand(char channel, unsigned short pulseWidth, unsigned int speed = 0);

    /**
     * @brief Create a string of the command for Serial use
     *
     * @return Command as string
     */
    std::string toString(unsigned short time = 0) const;
private:
    char channel; ///< Servo channel
    unsigned short pulseWidth; ///< Pulse width
    unsigned int speed; ///< Speed
};

/**
 * @brief ServoCommands stores multiple commands for servos
 */
class ServoCommands
{
public:
    /**
     * @brief ServoCommands constructor
     *
     * @param commands Vector of ServoCommand
     * @param time Duration that the execution of the commands should take
     */
    ServoCommands(std::vector<ServoCommand> commands, unsigned short time);

    /**
     * @brief Create a string of the commands for Serial use
     *
     * @return Commands as string
     */
    std::string toString() const;

    unsigned short getTime() const;

private:
    std::vector<ServoCommand> commands; ///< The commands
    unsigned short time; ///< The duration of execution
};

#endif //ROBOTARM_SERVOCOMMAND_HPP
