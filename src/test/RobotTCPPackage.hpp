#pragma once

#include <Elite/PrimaryPackage.hpp>
#include <algorithm>
#include <type_traits>
#include <cstring>
#include <iostream>

class RobotTCPPackage : public ELITE::PrimaryPackage {
private:
    constexpr static int ROBOT_TCP_PKG_TYPE = 4;
public:
    RobotTCPPackage() : PrimaryPackage(ROBOT_TCP_PKG_TYPE) { }

    ~RobotTCPPackage() = default;

    uint32_t unpackUInt32(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(uint32_t)];
        std::copy(iter, iter + sizeof(uint32_t), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        uint32_t result;
        std::memcpy(&result, bytes, sizeof(uint32_t));
        return result;
    }

    double unpackDouble(const std::vector<uint8_t>::const_iterator& iter) {
        uint8_t bytes[sizeof(double)];
        std::copy(iter, iter + sizeof(double), bytes);

        std::reverse(std::begin(bytes), std::end(bytes));
        
        double result;
        std::memcpy(&result, bytes, sizeof(double));
        return result;
    }

    virtual void parser(int len, const std::vector<uint8_t>::const_iterator& iter) override {
        int offset = 0;
        std::cout << "Package len: " << unpackUInt32(iter + offset) << std::endl;
        offset += sizeof(uint32_t);

        std::cout << "Package type: " << (int)*(iter + offset) << std::endl;
        offset += sizeof(uint8_t);

        std::cout << "TCP Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Position Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation X:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Y:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

        std::cout << "TCP Offset Rotation Z:" << unpackDouble(iter + offset) << std::endl;
        offset += sizeof(double);

    }
};