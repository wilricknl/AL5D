/**
 * @file Serial.hpp
 * @brief Serial connection using Boost ASIO
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#ifndef ROBOTARM_SERIAL_HPP
#define ROBOTARM_SERIAL_HPP

#include <boost/asio.hpp>

/**
 * @brief Serial takes care of the serial connection
 */
class Serial
{
public:
    /**
     * @brief Serial constructor
     *
     * @param device USB device name
     * @param rate Baud rate
     * @param characterSize Character bit size
     */
    Serial(std::string const& device, unsigned int rate, unsigned int characterSize = 8);

    virtual ~Serial();

    /**
     * @brief Open the serial connection
     */
    void open();

    /**
     * @brief Check if serial connection is open
     *
     * @return `true` if Serial is open, else `false`
     */
    bool isOpen() const;

    /**
     * @brief Send command on serial
     *
     * @param command The command to be sent
     */
    void send(std::string const& command);

    /**
     * @brief Send cancel command on serial
     *
     * @param command The command to be canceled
     */
    void cancel(std::string const& command);

    /**
     * @brief Query if the robot is still moving
     *
     * @return '+' if moving, else '.'
     */
    char queryMovementStatus();

    /**
     * @brief Get the USB device name
     *
     * @return USB device name
     */
    const std::string &getDevice() const;

    /**
     * @brief Set new device name
     *
     * @param device The new device name
     */
    void setDevice(const std::string &device);
private:
    /**
     * @brief Write command with specified end of line to serial
     *
     * @param command The command to be written
     * @param eol The end of line character
     */
    void write(std::string const& command, char eol);
private:
    std::string device; ///< USB device name
    unsigned int rate; ///< Baud rate
    unsigned int characterSize; ///< The character size
    boost::asio::io_service ioservice; ///< Input output service
    boost::asio::serial_port serial; ///< Serial port
};

#endif //ROBOTARM_SERIAL_HPP
