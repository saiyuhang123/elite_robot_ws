#include <iostream>
#include <memory>
#include <string>
#include <Elite/DashboardClient.hpp>

using namespace ELITE;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./dashboard_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = argv[1];

    std::unique_ptr<DashboardClient> my_dashboard;
    my_dashboard.reset(new DashboardClient());

    if (!my_dashboard->connect(robot_ip)) {
        std::cout << "Could not connect to robot" << std::endl;
        return 1;
    } else {
        std::cout << "Connect to robot" << std::endl;
    }

    // Power on
    if (!my_dashboard->powerOn()) {
        std::cout << "Could not send Power on command" << std::endl;
        return 1;
    } else {
        std::cout << "Power on" << std::endl;
    }

    // Brake release
    if (!my_dashboard->brakeRelease()) {
        std::cout << "Could not send BrakeRelease command" << std::endl;
        return 1;
    } else {
        std::cout << "Brake release" << std::endl;
    }
    
    my_dashboard->disconnect();

    return 0;
}