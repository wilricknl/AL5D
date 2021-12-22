/**
 * @file IHighLevelDriver.hpp
 * @brief High level driver interface
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_IHIGHLEVELDRIVER_H
#define ROBOTARM_IHIGHLEVELDRIVER_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <queue>
#include <vector>

#include "ILowLevelDriver.hpp"
#include "ServoCommand.hpp"

/**
 * @brief Predefined positions for the robotarm
 */
enum class Position
{
    PARK,
    READY,
    STRAIGHTUP
};

/**
 * @brief High level driver interface
 */
class IHighLevelDriver
{
public:
    IHighLevelDriver(std::unique_ptr<ILowLevelDriver> lowLevelDriver)
        :lowLevelDriver(std::move(lowLevelDriver))
    {}

    virtual ~IHighLevelDriver(){}

    /**
     * @brief Initialize the robotarm
     */
    virtual void initialize() = 0;

    /**
     * @brief Set the robotarm to a predefined position
     *
     * @p position A predefined position
     * @p time The time the operation should take
     */
    virtual void position(Position position, unsigned short time) = 0;

    /**
     * @brief Set the robotarm to the position
     *
     * @param commands The servo commands
     */
    virtual void position(ServoCommands commands) = 0;

    /**
     * @brief Perform emergency stop
     */
    virtual void emergencyStop() = 0;

    /**
     * @brief Convert degrees to pulsewidth
     *
     * @param degrees The degrees
     * @param channel The servo number
     *
     * @return The pulsewidth
     */
    virtual unsigned int degreesToPw(signed short degrees, char channel) = 0;

    /**
     * @brief Add commands to the queue
     *
     * @param commands The commands that must be added
     */
    void addCommandsToQueue(ServoCommands commands)
    {
        queue.emplace(commands);
    }

    /**
     * @brief Check if the queue is empty
     *
     * @return `true` if empty, else `false`
     */
    bool isQueueEmpty() const
    {
        return queue.empty();
    }

    /**
     * @brief Run the next item in the queue
     *
     * @return The executed commands
     */
    ServoCommands runQueue()
    {
        if (not isQueueEmpty())
        {
            auto commands = queue.front();
            lowLevelDriver->servosPosition(commands);
            queue.pop();
            return commands;
        }
        return {{}, 0 };
    }

    /**
     * @brief Empty the queue
     */
    void emptyQueue()
    {
        while (not queue.empty())
            queue.pop();
    }

    /**
     * @brief Get a reference to the low level driver
     *
     * @return @ref lowLevelDriver
     */
    std::unique_ptr<ILowLevelDriver>& getLowLevelDriver()
    {
        return lowLevelDriver;
    }

    /**
     * @brief Convert Position enum to ServoCommands
     *
     * @param position The position
     * @param time The duration of the commands
     *
     * @return The position as ServoCommands structure
     */
    ServoCommands positionToServoCommands(Position position, unsigned short time)
    {
        switch (position)
        {
            case Position::STRAIGHTUP:
                return {
                        {
                                { 0, 1500 },
                                { 1, 1500 },
                                { 2, 600 },
                                { 3, 1500 },
                                { 4, 500 }
                        },
                        time
                };
            case Position::READY:
                return {
                        {
                                { 0, 1000 },
                                { 1, 1200 },
                                { 2, 1650 },
                                { 3, 1500 },
                                { 4, 500 }
                        },
                        time
                };
            case Position::PARK:
            default:
                return {
                        {
                                { 0, 1000 },
                                { 1, 1000 },
                                { 2, 1850 },
                                { 3, 900 },
                                { 4, 500 }
                        },
                        time
                };
        }
    }

private:
    std::unique_ptr<ILowLevelDriver> lowLevelDriver; ///< The low level driver
    std::queue<ServoCommands> queue; ///< The queue
};

#endif //ROBOTARM_IHIGHLEVELDRIVER_H
