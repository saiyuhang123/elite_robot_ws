// modbus_client.cpp
// 简单的 Modbus TCP 客户端，用于控制正压/负压启停及参数设置
// 事务ID(Transaction ID)按要求忽略，固定填0，不影响设备解析
//
// 编译方法:
//   Linux:   g++ -O2 -o modbus_client modbus_client.cpp
//   Windows: g++ -O2 -o modbus_client.exe modbus_client.cpp -lws2_32
//
// 用法:
//   ./modbus_client <设备IP> [端口, 默认502]
// 确保定义了支持 inet_pton 的 Windows 版本（0x0600 代表 Windows Vista，0x0601 代表 Windows 7）
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif

// #include <winsock2.h>
// #include <ws2tcpip.h> // inet_pton 定义在此头文件中

#include <iostream>
#include <cstring>
#include <cstdint>
#include <vector>
#include <string>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef SOCKET sock_t;
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    typedef int sock_t;
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
#endif

class ModbusTcpClient {
public:
    ModbusTcpClient() : sockfd_(INVALID_SOCKET) {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }

    ~ModbusTcpClient() {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
    }

    bool connectServer(const std::string& ip, uint16_t port) {
        sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd_ == INVALID_SOCKET) {
            std::cerr << "创建socket失败" << std::endl;
            return false;
        }

        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

        if (connect(sockfd_, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
            std::cerr << "连接服务器失败: " << ip << ":" << port << std::endl;
            return false;
        }
        std::cout << "已连接到 " << ip << ":" << port << std::endl;
        return true;
    }

    void disconnect() {
        if (sockfd_ != INVALID_SOCKET) {
#ifdef _WIN32
            closesocket(sockfd_);
#else
            close(sockfd_);
#endif
            sockfd_ = INVALID_SOCKET;
        }
    }

    // 写单个线圈 (功能码 0x05)
    bool writeSingleCoil(uint8_t slaveId, uint16_t address, bool value) {
        std::vector<uint8_t> frame = buildHeader(6); // 单元号+功能码+地址+值 = 6字节
        frame.push_back(slaveId);
        frame.push_back(0x05);
        frame.push_back((address >> 8) & 0xFF);
        frame.push_back(address & 0xFF);
        if (value) {
            frame.push_back(0xFF);
            frame.push_back(0x00);
        } else {
            frame.push_back(0x00);
            frame.push_back(0x00);
        }
        return sendFrame(frame);
    }

    // 写单个寄存器 (功能码 0x06)
    bool writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
        std::vector<uint8_t> frame = buildHeader(6);
        frame.push_back(slaveId);
        frame.push_back(0x06);
        frame.push_back((address >> 8) & 0xFF);
        frame.push_back(address & 0xFF);
        frame.push_back((value >> 8) & 0xFF);
        frame.push_back(value & 0xFF);
        return sendFrame(frame);
    }

    // ============ 与报文一一对应的业务封装 ============

    // 01 34 00 00 00 06 01 05 01 00 FF 00  启动正压
    bool startPositivePressure(uint8_t slaveId = 0x01) {
        return writeSingleCoil(slaveId, 0x0100, true);
    }

    // 01 35 00 00 00 06 01 05 01 00 00 00  正压松气
    bool releasePositivePressure(uint8_t slaveId = 0x01) {
        return writeSingleCoil(slaveId, 0x0100, false);
    }

    // 01 4D 00 00 00 06 01 05 01 01 FF 00  启动负压
    bool startNegativePressure(uint8_t slaveId = 0x01) {
        return writeSingleCoil(slaveId, 0x0101, true);
    }

    // 01 4F 00 00 00 06 01 05 01 01 00 00  负压松气
    bool releaseNegativePressure(uint8_t slaveId = 0x01) {
        return writeSingleCoil(slaveId, 0x0101, false);
    }

    // 设置气压上限 (寄存器地址 0x0306), 例: 150 / 80 / 90
    bool setPressureUpperLimit(uint16_t value, uint8_t slaveId = 0x01) {
        return writeSingleRegister(slaveId, 0x0306, value);
    }

    // 设置负压值 (寄存器地址 0x0307, 有符号数), 例: -80 / -50 / -30
    bool setNegativePressure(int16_t value, uint8_t slaveId = 0x01) {
        return writeSingleRegister(slaveId, 0x0307, static_cast<uint16_t>(value));
    }

private:
    sock_t sockfd_;

    // 构建MBAP头: 事务ID固定填0(按要求忽略), 协议ID固定0, length为后续字节数
    std::vector<uint8_t> buildHeader(uint16_t length) {
        std::vector<uint8_t> header;
        header.push_back(0x00); // 事务ID 高字节(忽略, 固定0)
        header.push_back(0x00); // 事务ID 低字节
        header.push_back(0x00); // 协议ID 高字节
        header.push_back(0x00); // 协议ID 低字节
        header.push_back((length >> 8) & 0xFF); // 长度 高字节
        header.push_back(length & 0xFF);        // 长度 低字节
        return header;
    }

    bool sendFrame(const std::vector<uint8_t>& frame) {
        if (sockfd_ == INVALID_SOCKET) {
            std::cerr << "未连接服务器" << std::endl;
            return false;
        }

        printHex("发送", frame);

        int sent = send(sockfd_, (const char*)frame.data(), (int)frame.size(), 0);
        if (sent == SOCKET_ERROR || sent != (int)frame.size()) {
            std::cerr << "发送失败" << std::endl;
            return false;
        }

        // 接收设备应答
        uint8_t buf[256];
        int recvLen = recv(sockfd_, (char*)buf, sizeof(buf), 0);
        if (recvLen > 0) {
            printHex("接收", std::vector<uint8_t>(buf, buf + recvLen));
            return true;
        }
        std::cerr << "未收到应答" << std::endl;
        return false;
    }

    void printHex(const std::string& tag, const std::vector<uint8_t>& data) {
        std::cout << tag << ": ";
        char tmp[4];
        for (auto b : data) {
            snprintf(tmp, sizeof(tmp), "%02X ", b);
            std::cout << tmp;
        }
        std::cout << std::endl;
    }
};

int main(int argc, char* argv[]) {
    std::string ip = "192.168.1.200"; // 默认IP，按实际设备修改
    uint16_t port = 502;             // Modbus TCP 默认端口

    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = static_cast<uint16_t>(std::stoi(argv[2]));

    ModbusTcpClient client;
    if (!client.connectServer(ip, port)) {
        return -1;
    }

    // ===== 使用示例，按需取消注释 =====

    //client.startPositivePressure();        // 启动正压
     client.releasePositivePressure();   // 正压松气
    // client.startNegativePressure();     // 启动负压
    // client.releaseNegativePressure();   // 负压松气

    // client.setPressureUpperLimit(150);  // 设置气压上限150
    // client.setPressureUpperLimit(80);   // 设置气压上限80
    // client.setPressureUpperLimit(90);   // 设置气压上限90

    // client.setNegativePressure(-80);    // 设置负压-80
    // client.setNegativePressure(-50);    // 设置负压-50
    //client.setNegativePressure(-30);    // 设置负压-30

    client.disconnect();
    return 0;
}
