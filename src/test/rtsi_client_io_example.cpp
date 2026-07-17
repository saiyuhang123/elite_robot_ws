#include <iostream>
#include <memory>
#include <chrono>

#include <Elite/RtsiIOInterface.hpp>

using namespace ELITE;
using namespace std::chrono;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./rtsi_client_io_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = std::string(argv[1]);

    std::unique_ptr<RtsiIOInterface> io_interface = std::make_unique<RtsiIOInterface>("joint_recipe.txt", "digital_recipe.txt", 250);

    if (!io_interface->connect(robot_ip)) {
        std::cout << "Couldn't connect RTSI server" << std::endl;
        return 1;
    }

    VersionInfo version = io_interface->getControllerVersion();
    std::cout << "Controller is: " << version.toString() << std::endl;

    int count = 250;
    auto next = steady_clock::now();
    while(count--) {
        auto actula_joints = io_interface->getActualJointPositions();
        auto timestamp = io_interface->getTimestamp();

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actula_joints) {
            std::cout << i << " ";
        }
        std::cout << std::endl;

        next += 4ms;
        std::this_thread::sleep_until(next);
    }

    io_interface->setStandardDigital(0, 1);
    std::this_thread::sleep_for(100ms);

    io_interface->disconnect();

    return 0;
}