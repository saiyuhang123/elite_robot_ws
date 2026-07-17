#include <iostream>
#include <memory>
#include <string>
#include <Elite/RtsiClientInterface.hpp>
#include <Elite/RtsiRecipe.hpp>

using namespace ELITE;


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Must provide robot IP. Example: ./rtsi_client_example aaa.bbb.ccc.ddd" << std::endl;
        return 1;
    }
    std::string robot_ip = std::string(argv[1]);

    std::unique_ptr<RtsiClientInterface> rtsi = std::make_unique<RtsiClientInterface>();

    rtsi->connect(robot_ip);

    if(rtsi->negotiateProtocolVersion()) {
        std::cout << "Negotiate protocol version success" << std::endl;
    } else {
        std::cout << "Negotiate protocol version fail" << std::endl;
        return 1;
    }
    
    std::cout << "Controller version: " << rtsi->getControllerVersion().toString() << std::endl;

    auto out_recipe = rtsi->setupOutputRecipe({"timestamp", "actual_joint_positions"}, 250);

    auto in_recipe = rtsi->setupInputRecipe({"standard_digital_output_mask", "standard_digital_output"});

    if(rtsi->start()) {
        std::cout << "RTSI sync start successful" << std::endl;
    } else {
        std::cout << "RTSI sync start fail" << std::endl;
        return 1;
    }
    
    double timestamp;
    vector6d_t actula_joints;
    int count = 250;
    while(count--) {
        if (!rtsi->receiveData(out_recipe)) {
            std::cout << "Receive recipe fail" << std::endl;
            return 1;
        }
        out_recipe->getValue("timestamp", timestamp);
        out_recipe->getValue("actual_joint_positions", actula_joints);

        std::cout << "timestamp: " << timestamp << std::endl;
        std::cout << "actual_joint_positions: ";
        for(auto i : actula_joints) {
            std::cout << i << " ";
        }
        std::cout << std::endl;
    }

    in_recipe->setValue("standard_digital_output_mask", 1);
    in_recipe->setValue("standard_digital_output", 1);
    rtsi->send(in_recipe);

    rtsi->disconnect();

    return 0;
}