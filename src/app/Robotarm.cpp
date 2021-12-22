/**
 * @file Robotarm.cpp
 * @author sandraak (https://github.com/sandraak)
 * @author wilricknl (https://github.com/wilricknl)
 */
#include "app/Robotarm.hpp"

Robotarm::Robotarm(std::unique_ptr<ILowLevelDriver> lowLevelDriver)
    : Node("RobotarmNode"), IHighLevelDriver(std::move(lowLevelDriver))
{
    queueTimer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Robotarm::queueTimerCallback, this)
    );

    subscriber = this->create_subscription<std_msgs::msg::String>(
            "command",
            10,
            std::bind(&Robotarm::commandCallback, this, std::placeholders::_1)
        );
    publisherInfo = this->create_publisher<std_msgs::msg::String>("info", 10);
    publisherDebug = this->create_publisher<std_msgs::msg::String>("debug", 10);
    positionSubscriber = this->create_subscription<robotarm::msg::Position>(
            "custom",
            10,
            std::bind(&Robotarm::positionCallback, this, std::placeholders::_1));
    initialize();
}

void Robotarm::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string logMessage{ "Bewegen" };

    if (msg->data == "init")
    {
        initialize();
        return;
    }
    else if(msg->data == "stop"){
        logMessage = "Noodstop";
        emergencyStop();
    }
    if(bInitialized) {
        if (msg->data == "park") {
            addCommandsToQueue(positionToServoCommands(Position::PARK, 1200));
        } else if (msg->data == "ready") {
            addCommandsToQueue(positionToServoCommands(Position::READY, 1200));
        } else if (msg->data == "straightup") {
            addCommandsToQueue(positionToServoCommands(Position::STRAIGHTUP, 1200));
        }
    }
    publishMessage(msg->data, Logtype::DEBUG);
    publishMessage(logMessage, Logtype::INFO);
}

void Robotarm::initialize() {
    bInitialized = true;
    getLowLevelDriver()->servosPosition(positionToServoCommands(Position::PARK, 6000));
    publishMessage("Initialisatie", Logtype::INFO);
}

void Robotarm::position(Position position, unsigned short time) {
    publishMessage("Bewegen_position", Logtype::INFO);
    getLowLevelDriver()->servosPosition(positionToServoCommands(position, time));
}
void Robotarm::position(ServoCommands commands){
    getLowLevelDriver()->servosPosition(std::move(commands));
}

void Robotarm::emergencyStop() {
    bInitialized = false;
    for(int i = 0; i< 5; ++i) {
        getLowLevelDriver()->stopServo(i);
    }
    emptyQueue();
}

unsigned int Robotarm::degreesToPw(short degrees, int minDegrees, int maxDegrees, int minPulseWidth, int maxPulseWidth) {
    return (std::min(std::max(minDegrees, (signed int)degrees), maxDegrees) - minDegrees)
        / (maxDegrees - minDegrees)
        * (maxPulseWidth - minPulseWidth)
        + minPulseWidth;
}

unsigned int Robotarm::degreesToPw(signed short degrees, char channel){
    unsigned int pulseWidth = 0;
    switch(channel) {
        case 0:
            pulseWidth = degreesToPw(degrees, -90, 90, 600, 2500);
            break;
        case 1:
            pulseWidth = degreesToPw(degrees, -30, 90, 700, 1500);
            break;
        case 2:
            pulseWidth = degreesToPw(degrees, 90, 135, 1500, 2000);
            break;
        case 3:
            pulseWidth = degreesToPw(degrees, -90, 90, 500, 2500);
            break;
        case 4:
            pulseWidth = degreesToPw(degrees, 90, 135, 2000, 2500);
            break;
        case 5:
            pulseWidth = degreesToPw(degrees, 90, 135, 2000, 2500);
            break;
        default:
            break;
    }
    return pulseWidth;
}

std::string Robotarm::positionToString(Position position){
    std::string positionString;
    switch (position) {
        case Position::PARK:
            positionString = "PARK";
            break;
        case Position::READY:
            positionString = "READY";
            break;
        case Position::STRAIGHTUP:
            positionString ="STRAIGHTUP";
            break;
        default:
            break;
    }
    return positionString;
}

void Robotarm::publishMessage(std::string msg, Logtype logtype){
    auto message = std_msgs::msg::String();
    message.data = msg;
    if(logtype == Logtype::INFO) {
        publisherInfo->publish(message);
    }
    else if (logtype == Logtype::DEBUG){
        publisherDebug->publish(message);
    }
}

ServoCommands Robotarm::messageToCommand(const robotarm::msg::Position::SharedPtr message){
   std::vector<ServoCommand> commands;
   if(message->base != 404){
       commands.emplace_back(ServoCommand(0, degreesToPw(message->base, 0)));
   }
   if(message->shoulder != 404){
       commands.emplace_back(ServoCommand(1,  degreesToPw(message->shoulder, 1)));
   }
    if(message->elbow != 404){
        commands.emplace_back(ServoCommand(2,  degreesToPw(message->elbow, 2)));
    }
    if(message->wrist != 404){
        commands.emplace_back(ServoCommand(3,  degreesToPw(message->wrist, 3)));
    }
    if(message->gripper != 404){
        commands.emplace_back(ServoCommand(4,  degreesToPw(message->gripper,4)));
    }
    if(message->rotate != 404){
        commands.emplace_back(ServoCommand(5,  degreesToPw(message->rotate,5)));
    }
    return {commands, (unsigned short)message->time};
}

void Robotarm::positionCallback(const robotarm::msg::Position::SharedPtr message) {
    if(bInitialized) {
        publishMessage("Bewegen position", Logtype::INFO);
        addCommandsToQueue(messageToCommand(message));
    }
}

void Robotarm::queueTimerCallback() {
    static bool bPrint = true; // Check om paraat status printen
    bool bMoving = getLowLevelDriver()->isMoving();

    if (not bMoving and bPrint)
    {
        publishMessage("Paraat", Logtype::INFO);
        bPrint = false;
    }

    if (not bMoving and not isQueueEmpty())
    {
        auto commands = runQueue();
        publishMessage("Servos bewegen naar volgende posities : " + commands.toString(), Logtype::INFO);
        bPrint = true;
    }
}
