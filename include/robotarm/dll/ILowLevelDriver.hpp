/**
 * @file ILowLevelDriver.hpp
 * @brief The low level driver interface
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_ILOWLEVELDRIVER_HPP
#define ROBOTARM_ILOWLEVELDRIVER_HPP

#include "Serial.hpp"
#include "ServoCommand.hpp"

/**
 * @brief The low level driver interface
 */
class ILowLevelDriver
{
public:
    /**
     * @brief ILowLevelDriver constructor
     *
     * @param device USB device name
     * @param rate Baud rate
     * @param characterSize Character bit size
     */
    ILowLevelDriver(std::string const& device, unsigned int rate, unsigned int characterSize = 8)
            : device(device), rate(rate), characterSize(characterSize), serial(device, rate, characterSize)
    {}

    virtual ~ILowLevelDriver() = default;

    /**
     * @brief Perform a single command
     *
     * @param command The command to be performed
     * @param time The duration of the command
     */
    virtual void servoPosition(ServoCommand command, unsigned short time) = 0;

    /**
     * @brief Perform multiple commands at once
     *
     * @param commands The commdands to be performed
     */
    virtual void servosPosition(ServoCommands commands) = 0;

    /**
     * @brief Check if robotarm is moving
     *
     * @return `true` if moving, else `false`
     */
    virtual bool isMoving() = 0;

    /**
     * @brief Stop a servo
     *
     * @param channel The servo to be stopped
     */
    virtual void stopServo(char channel) = 0;

    /**
     * @brief Get access to the serial class
     *
     * @return Reference to serial
     */
    Serial& getSerial()
    {
        return serial;
    }
private:
    std::string device; ///< USB port name
    unsigned int rate; ///< Baud rate
    unsigned int characterSize; ///< Character bit size
    Serial serial; ///< Serial connection
};

#endif //ROBOTARM_ILOWLEVELDRIVER_HPP
