/**
 * @file Serial.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "../../robotarm/dll/Serial.hpp"

Serial::Serial(const std::string &device, unsigned int rate, unsigned int characterSize)
    : device(device), rate(rate), characterSize(characterSize), serial(ioservice, device)
{}

Serial::~Serial() {
    if (isOpen())
    {
        serial.close();
    }
}

void Serial::open()
{
    if (not serial.is_open())
    {
        serial = boost::asio::serial_port(ioservice, device);
        serial.set_option(boost::asio::serial_port_base::baud_rate(rate));
        serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(characterSize)));
    }
}

bool Serial::isOpen() const
{
    return serial.is_open();
}

void Serial::send(std::string const& command)
{
    write(command, '\r');
}

void Serial::cancel(std::string const& command)
{
    write(command, '\x27');
}

const std::string &Serial::getDevice() const {
    return device;
}

void Serial::setDevice(const std::string &device) {
    Serial::device = device;
}

void Serial::write(std::string const& command, char eol)
{
    boost::asio::streambuf buffer;
    std::ostream os(&buffer);
    os << command << eol;
    boost::asio::write(serial, buffer.data());
    os.flush();
}

char Serial::queryMovementStatus() {
    write("Q", '\r');

    char data = 0;
    boost::asio::read(serial, boost::asio::buffer(&data, 1));

    return data;
}
