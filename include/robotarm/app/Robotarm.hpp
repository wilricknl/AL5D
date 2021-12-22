/**
 * @file Robotarm.hpp
 * @brief The high level driver
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_ROBOTARM_HPP
#define ROBOTARM_ROBOTARM_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robotarm/msg/position.hpp"

#include "IHighLevelDriver.hpp"
#include "Logger.hpp"

/**
 * @brief The high level driver
 */
class Robotarm : public rclcpp::Node, public IHighLevelDriver
{
public:
    Robotarm(std::unique_ptr<ILowLevelDriver> lowLevelDriver);

    /**
     * @brief Initialize the robotarm
     */
    void initialize() override;

    /**
     * @brief Set the robotarm to a predefined position
     *
     * @p position A predefined position
     * @p time The time the operation should take
     */
    void position(Position position, unsigned short time) override;

    /**
     * @brief Set the robotarm to the position
     *
     * @param commands The servo commands
     */
    void position(ServoCommands commands) override;

    /**
     * @brief Perform emergency stop
     */
    void emergencyStop() override;

    /**
     * @brief Degrees to pulsewidth mapper
     *
     * @param degrees Degrees
     * @param minDegrees Minimum degrees
     * @param maxDegrees Maximum degrees
     * @param minPulseWidth Minimum pulsewidth
     * @param maxPulseWidth Maximum pulsewidth
     *
     * @return Pulsewidth
     */
    unsigned int degreesToPw(signed short degrees, int minDegrees, int maxDegrees, int minPulseWidth, int maxPulseWidth);

    /**
     * @brief Convert degrees to pulsewidth
     *
     * @param degrees The degrees
     * @param channel The servo number
     *
     * @return The pulsewidth
     */
    unsigned int degreesToPw(signed short degrees, char channel) override;
private:
    /**
     * @brief The command callback
     *
     * @param message The command
     */
    void commandCallback(const std_msgs::msg::String::SharedPtr message);

    /**
     * @brief Convert a predefined position to string
     *
     * @param position Predefined position
     * @return String
     */
    std::string positionToString(Position position);

    /**
     * @brief Publish log message
     *
     * @param message The message to be logged
     * @param logtype Log level
     */
    void publishMessage(std::string message, Logtype logtype);

    /**
     * @brief Queue timer callback
     */
    void queueTimerCallback();

    /**
     * @brief Convert a custom position message to ServoCommands
     *
     * @param message Position data
     * @return ServoCommands structure for the position
     */
    ServoCommands messageToCommand(const robotarm::msg::Position::SharedPtr message);

    /**
     * @brief The custom position callback
     *
     * @param message The position data
     */
    void positionCallback(const robotarm::msg::Position::SharedPtr message);
private:
    rclcpp::TimerBase::SharedPtr queueTimer; ///< Queue callback timer
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber; ///< Subscriber to common commands
    rclcpp::Subscription<robotarm::msg::Position>::SharedPtr positionSubscriber; ///< Subscriber to custom position commands
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherInfo; ///< Info log publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherDebug; ///< Debug log publisher
    bool bInitialized; ///<initialization status
};

#endif //ROBOTARM_ROBOTARM_HPP
