
// #include <unistd.h>
// #include <cstdlib>
// #include <iostream>
// #include <stdio.h>
// #include <cstring>
// #include <unistd.h>
#include "frcobot_hw/frcobot_hw.h"
#include "frcobot_hw/FRRobot.h"
#include "frcobot_hw/RobotError.h"
#include "frcobot_hw/RobotTypes.h"

#include "xmlrpc-c/base.h"
#include "xmlrpc-c/client.h"

using namespace std;

int main(int argc, char** argv)
{
    FRRobot robot;                 //Instantiate the robot object
    robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

    int err = robot.ResetAllError();
    std::cout << "Error code: " << err << std::endl;
    return 0;
}