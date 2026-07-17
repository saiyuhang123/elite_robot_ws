#include <Elite/PrimaryPortInterface.hpp>
#include "RobotTCPPackage.hpp"
#include <memory>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono;

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./primary_pkg_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    auto primary = std::make_unique<ELITE::PrimaryPortInterface>();

    auto robotPackage = std::make_shared<RobotTCPPackage>();

    primary->connect(robot_ip, 30001);

    primary->getPackage(robotPackage, 200);

    primary->disconnect();

    return 0;
}