/**
 * @file SSC32.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "../../include/robotarm/dll/SSC32.hpp"
#include <iostream>

SSC32::SSC32(const std::string &device, unsigned int rate, unsigned int characterSize)
        : ILowLevelDriver(device, rate,characterSize)
{}

SSC32::~SSC32() {

}

void SSC32::servoPosition(ServoCommand command, unsigned short time) {
    getSerial().send(command.toString(time));
}

void SSC32::servosPosition(ServoCommands commands)
{
    getSerial().send(commands.toString());
}

void SSC32::stopServo(char channel) {
    std::string command = "STOP" + std::to_string(channel);
    getSerial().send(command);
}

bool SSC32::isMoving() {
    return getSerial().queryMovementStatus() == '+';
}
