/**
 * 一次性工具：从控制器读取 CS66 的 MDH 运动学参数（出厂标定值，恒定）
 * 编译: g++ -o read_mdh read_mdh.cpp -lelite-cs-series-sdk \
 *       -I<workspace>/build/elite_cs_series_sdk/include -L/lib/aarch64-linux-gnu
 * 用法: ./read_mdh <robot_ip>
 */
#include <Elite/PrimaryPortInterface.hpp>
#include <Elite/RobotConfPackage.hpp>
#include <iostream>

using namespace ELITE;

int main(int argc, const char** argv) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <robot_ip>" << std::endl;
        return 1;
    }
    auto primary = std::make_unique<PrimaryPortInterface>();
    auto kin_info = std::make_shared<KinematicsInfo>();
    if (!primary->connect(argv[1], 30001)) {
        std::cerr << "连接控制器 30001 端口失败" << std::endl;
        return 1;
    }
    if (!primary->getPackage(kin_info, 200)) {
        std::cerr << "读取运动学参数失败" << std::endl;
        return 1;
    }
    primary->disconnect();

    auto print = [](const char* name, const vector6d_t& v) {
        std::cout << name << " = {";
        for (size_t i = 0; i < v.size(); i++) {
            std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
        }
        std::cout << "}" << std::endl;
    };
    print("dh_alpha", kin_info->dh_alpha_);
    print("dh_a", kin_info->dh_a_);
    print("dh_d", kin_info->dh_d_);
    return 0;
}
