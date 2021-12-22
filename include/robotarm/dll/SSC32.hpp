/**
 * @file SSC32.hpp
 * @brief The low level driver
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_SSC32_H
#define ROBOTARM_SSC32_H

#include "ILowLevelDriver.hpp"

/**
 * @brief The low level driver
 */
class SSC32 : public ILowLevelDriver
{
public:
    /**
     * @brief SSC32 constructor
     *
     * @param device USB device name
     * @param rate Baud rate
     * @param characterSize Character bit size
     */
    SSC32(const std::string &device, unsigned int rate, unsigned int characterSize = 8);
    ~SSC32();

    /**
     * @brief Perform a single command
     *
     * @param command The command to be performed
     * @param time The duration of the command
     */
    void servoPosition(ServoCommand command, unsigned short time) override;

    /**
     * @brief Perform multiple commands at once
     *
     * @param commands The commdands to be performed
     */
    void servosPosition(ServoCommands commands) override;

    /**
     * @brief Stop a servo
     *
     * @param channel The servo to be stopped
     */
    void stopServo(char channel) override;

    /**
     * @brief Check if robotarm is moving
     *
     * @return `true` if moving, else `false`
     */
    virtual bool isMoving() override;
};

#endif //ROBOTARM_SSC32_H
