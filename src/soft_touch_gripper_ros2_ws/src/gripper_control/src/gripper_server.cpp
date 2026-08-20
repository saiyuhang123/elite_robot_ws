// gripper_server.cpp
// ROS2 服务节点 —— 通过 Modbus TCP 控制软体抓手正压/负压启停及参数设置
//
// 基于 src/modbus.cpp 改写为 ROS2 Service 模式。
// 提供 GripperCommand.srv 服务 + /gripper_pressure 话题轮询实时气压。
//
// 编译: colcon build --packages-select gripper_control
// 运行: ros2 run gripper_control gripper_server
// 调用示例:
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 0, value: 0, slave_id: 1}"       # 启动正压
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 2, value: 0, slave_id: 1}"       # 启动负压
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 4, value: 150, slave_id: 1}"     # 设置气压上限150
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 5, value: -50, slave_id: 1}"     # 设置负压-50
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 6, value: 0, slave_id: 1}"       # 查正压反馈
//   ros2 service call /gripper_command gripper_control/srv/GripperCommand \
//     "{command: 7, value: 0, slave_id: 1}"       # 查负压反馈
//
// 监听气压话题:
//   ros2 topic echo /gripper_pressure

// 事务ID(Transaction ID)按要求忽略，固定填0，不影响设备解析

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "gripper_control/srv/gripper_command.hpp"

using GripperCommand = gripper_control::srv::GripperCommand;

typedef int sock_t;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1

// ============================================================
// 命令常量
// ============================================================
namespace GripperCmd {
  constexpr uint8_t START_POSITIVE_PRESSURE    = 0;
  constexpr uint8_t RELEASE_POSITIVE_PRESSURE  = 1;
  constexpr uint8_t START_NEGATIVE_PRESSURE    = 2;
  constexpr uint8_t RELEASE_NEGATIVE_PRESSURE  = 3;
  constexpr uint8_t SET_PRESSURE_UPPER_LIMIT   = 4;
  constexpr uint8_t SET_NEGATIVE_PRESSURE      = 5;
  constexpr uint8_t READ_POSITIVE_FEEDBACK     = 6;
  constexpr uint8_t READ_NEGATIVE_FEEDBACK     = 7;
}

// ============================================================
// 寄存器地址定义
// ============================================================
namespace RegAddr {
  // 保持寄存器 (功能码 0x03)
  constexpr uint16_t PRESSURE_INT16   = 770;   // 0x0302, 气压值(有符号整数 kPa)
  constexpr uint16_t PRESSURE_FLOAT32 = 780;   // 气压值(浮点数, 占2寄存器)

  // 离散输入 (功能码 0x02)
  constexpr uint16_t POS_FEEDBACK = 512;       // 0x0200, 正压反馈 0/1
  constexpr uint16_t NEG_FEEDBACK = 513;       // 0x0201, 负压反馈 0/1
}

// ============================================================
// Modbus TCP 客户端
// ============================================================
class ModbusTcpClient {
public:
  ModbusTcpClient() : sockfd_(INVALID_SOCKET) {}

  ~ModbusTcpClient() {
    disconnect();
  }

  bool connectServer(const std::string& ip, uint16_t port) {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    if (sockfd_ != INVALID_SOCKET) {
      return true;
    }

    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ == INVALID_SOCKET) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "创建socket失败");
      return false;
    }

    // 设置收/发超时 500ms，避免设备异常时阻塞调用线程
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &serverAddr.sin_addr);

    if (connect(sockfd_, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"),
        "连接服务器失败: %s:%d", ip.c_str(), port);
      close(sockfd_);
      sockfd_ = INVALID_SOCKET;
      return false;
    }
    RCLCPP_INFO(rclcpp::get_logger("modbus"),
      "已连接到 %s:%d", ip.c_str(), port);
    return true;
  }

  void disconnect() {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    if (sockfd_ != INVALID_SOCKET) {
      close(sockfd_);
      sockfd_ = INVALID_SOCKET;
    }
  }

  bool isConnected() const {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    return sockfd_ != INVALID_SOCKET;
  }

  // ========== 写操作 ==========

  // 写单个线圈 (功能码 0x05)
  bool writeSingleCoil(uint8_t slaveId, uint16_t address, bool value) {
    std::vector<uint8_t> frame = buildHeader(6);
    frame.push_back(slaveId);
    frame.push_back(0x05);
    frame.push_back((address >> 8) & 0xFF);
    frame.push_back(address & 0xFF);
    frame.push_back(value ? 0xFF : 0x00);
    frame.push_back(value ? 0x00 : 0x00);
    return sendAndRecv(frame);
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
    return sendAndRecv(frame);
  }

  // ========== 读操作 ==========

  // 读保持寄存器 (功能码 0x03)
  std::vector<uint16_t> readHoldingRegisters(uint8_t slaveId,
                                             uint16_t startAddress,
                                             uint16_t quantity) {
    std::vector<uint8_t> frame = buildHeader(6);
    frame.push_back(slaveId);
    frame.push_back(0x03);
    frame.push_back((startAddress >> 8) & 0xFF);
    frame.push_back(startAddress & 0xFF);
    frame.push_back((quantity >> 8) & 0xFF);
    frame.push_back(quantity & 0xFF);

    std::vector<uint8_t> resp;
    if (!sendAndRecv(frame, &resp)) {
      return {};
    }

    if (resp.size() < 9) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "应答报文过短");
      return {};
    }
    uint8_t byteCount = resp[8];
    if (resp.size() < 9U + byteCount) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "应答数据不完整");
      return {};
    }

    std::vector<uint16_t> registers;
    for (int i = 0; i < byteCount; i += 2) {
      uint16_t reg = (static_cast<uint16_t>(resp[9 + i]) << 8)
                   |  static_cast<uint16_t>(resp[9 + i + 1]);
      registers.push_back(reg);
    }
    return registers;
  }

  // 读离散输入 (功能码 0x02)
  // 返回每个输入点的 bool 值，失败返回空 vector
  std::vector<bool> readDiscreteInputs(uint8_t slaveId,
                                       uint16_t startAddress,
                                       uint16_t quantity) {
    std::vector<uint8_t> frame = buildHeader(6);
    frame.push_back(slaveId);
    frame.push_back(0x02);
    frame.push_back((startAddress >> 8) & 0xFF);
    frame.push_back(startAddress & 0xFF);
    frame.push_back((quantity >> 8) & 0xFF);
    frame.push_back(quantity & 0xFF);

    std::vector<uint8_t> resp;
    if (!sendAndRecv(frame, &resp)) {
      return {};
    }

    // 解析响应:
    // [0..5] MBAP头
    // [6]    从站ID
    // [7]    功能码 0x02
    // [8]    字节计数 N
    // [9..]  输入状态 (每字节8路, 低位在前)
    if (resp.size() < 9) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "应答报文过短");
      return {};
    }
    uint8_t byteCount = resp[8];
    if (resp.size() < 9U + byteCount) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "应答数据不完整");
      return {};
    }

    std::vector<bool> inputs;
    for (uint16_t i = 0; i < quantity; i++) {
      uint8_t byteVal = resp[9 + i / 8];
      inputs.push_back((byteVal >> (i % 8)) & 0x01);
    }
    return inputs;
  }

  // 读取气压值 (整数型, 地址 770)
  bool readPressureInt16(int16_t& value, uint8_t slaveId = 0x01) {
    auto regs = readHoldingRegisters(slaveId, RegAddr::PRESSURE_INT16, 1);
    if (regs.empty()) return false;
    value = static_cast<int16_t>(regs[0]);
    return true;
  }

  // 读取气压值 (浮点型, 地址 780, 占2个寄存器)
  bool readPressureFloat(float& value, uint8_t slaveId = 0x01) {
    auto regs = readHoldingRegisters(slaveId, RegAddr::PRESSURE_FLOAT32, 2);
    if (regs.size() < 2) return false;
    uint32_t bits = (static_cast<uint32_t>(regs[0]) << 16)
                  |  static_cast<uint32_t>(regs[1]);
    std::memcpy(&value, &bits, sizeof(float));
    return true;
  }

  // 读取正压反馈 (离散输入 0x0200, 返回 0 或 1)
  bool readPositiveFeedback(bool& value, uint8_t slaveId = 0x01) {
    auto inputs = readDiscreteInputs(slaveId, RegAddr::POS_FEEDBACK, 1);
    if (inputs.empty()) return false;
    value = inputs[0];
    return true;
  }

  // 读取负压反馈 (离散输入 0x0201, 返回 0 或 1)
  bool readNegativeFeedback(bool& value, uint8_t slaveId = 0x01) {
    auto inputs = readDiscreteInputs(slaveId, RegAddr::NEG_FEEDBACK, 1);
    if (inputs.empty()) return false;
    value = inputs[0];
    return true;
  }

  // ============ 业务封装 (写操作) ============

  bool startPositivePressure(uint8_t slaveId = 0x01) {
    return writeSingleCoil(slaveId, 0x0100, true);
  }

  bool releasePositivePressure(uint8_t slaveId = 0x01) {
    return writeSingleCoil(slaveId, 0x0100, false);
  }

  bool startNegativePressure(uint8_t slaveId = 0x01) {
    return writeSingleCoil(slaveId, 0x0101, true);
  }

  bool releaseNegativePressure(uint8_t slaveId = 0x01) {
    return writeSingleCoil(slaveId, 0x0101, false);
  }

  bool setPressureUpperLimit(uint16_t value, uint8_t slaveId = 0x01) {
    return writeSingleRegister(slaveId, 0x0306, value);
  }

  bool setNegativePressure(int16_t value, uint8_t slaveId = 0x01) {
    return writeSingleRegister(slaveId, 0x0307, static_cast<uint16_t>(value));
  }

private:
  sock_t sockfd_;
  mutable std::recursive_mutex io_mutex_;  // Modbus socket 并发访问保护

  std::vector<uint8_t> buildHeader(uint16_t length) {
    std::vector<uint8_t> header;
    header.push_back(0x00);
    header.push_back(0x00);
    header.push_back(0x00);
    header.push_back(0x00);
    header.push_back((length >> 8) & 0xFF);
    header.push_back(length & 0xFF);
    return header;
  }

  bool sendAndRecv(const std::vector<uint8_t>& frame) {
    std::vector<uint8_t> dummy;
    return sendAndRecv(frame, &dummy);
  }

  bool sendAndRecv(const std::vector<uint8_t>& frame,
                   std::vector<uint8_t>* response) {
    std::lock_guard<std::recursive_mutex> lock(io_mutex_);
    if (sockfd_ == INVALID_SOCKET) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "未连接服务器");
      return false;
    }

    printHex("发送", frame);

    int sent = send(sockfd_, (const char*)frame.data(), (int)frame.size(), 0);
    if (sent == SOCKET_ERROR || sent != (int)frame.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("modbus"), "发送失败");
      return false;
    }

    uint8_t buf[256];
    int recvLen = recv(sockfd_, (char*)buf, sizeof(buf), 0);
    if (recvLen > 0) {
      if (response) {
        response->assign(buf, buf + recvLen);
      }
      printHex("接收", std::vector<uint8_t>(buf, buf + recvLen));
      return true;
    }
    RCLCPP_ERROR(rclcpp::get_logger("modbus"), "未收到应答");
    return false;
  }

  void printHex(const std::string& tag, const std::vector<uint8_t>& data) {
    std::ostringstream oss;
    oss << tag << ": ";
    char tmp[4];
    for (auto b : data) {
      snprintf(tmp, sizeof(tmp), "%02X ", b);
      oss << tmp;
    }
    // 气压轮询逐帧 INFO 会刷屏并占 CPU；需要排查 Modbus 时再开 debug 日志。
    RCLCPP_DEBUG(rclcpp::get_logger("modbus"), "%s", oss.str().c_str());
  }
};

// ============================================================
// ROS2 服务节点
// ============================================================
class GripperServer : public rclcpp::Node {
public:
  GripperServer() : Node("gripper_server") {
    this->declare_parameter<std::string>("device_ip", "192.168.1.194");
    this->declare_parameter<int>("device_port", 502);
    this->declare_parameter<double>("poll_rate_hz", 5.0);

    std::string ip = this->get_parameter("device_ip").as_string();
    int port = this->get_parameter("device_port").as_int();
    double rate = this->get_parameter("poll_rate_hz").as_double();

    if (!client_.connectServer(ip, static_cast<uint16_t>(port))) {
      RCLCPP_WARN(this->get_logger(),
        "启动时无法连接设备 %s:%d, 将在服务调用时自动重连", ip.c_str(), port);
    }

    // 服务与气压轮询放入不同 callback group，由 MultiThreadedExecutor 并行处理，
    // 避免阻塞式 Modbus 轮询饿死 /gripper_command 服务回调。
    service_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    poll_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // 服务
    service_ = this->create_service<GripperCommand>(
      "gripper_command",
      std::bind(&GripperServer::handleCommand, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      service_cb_group_);

    // 气压话题 (轮询)
    pressure_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "gripper_pressure", 10);
    pressure_info_pub_ = this->create_publisher<std_msgs::msg::String>(
      "gripper_pressure_info", 10);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
    poll_timer_ = this->create_wall_timer(
      period,
      std::bind(&GripperServer::pollPressure, this),
      poll_cb_group_);

    RCLCPP_INFO(this->get_logger(),
      "抓手控制服务已就绪\n"
      "  服务: /gripper_command (command 0~7)\n"
      "  话题: /gripper_pressure (Float32, kPa)\n"
      "  轮询: %.1f Hz", rate);
  }

private:
  ModbusTcpClient client_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr poll_cb_group_;
  rclcpp::Service<GripperCommand>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pressure_info_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  std::mutex connect_mutex_;

  bool ensureConnected() {
    std::lock_guard<std::mutex> lock(connect_mutex_);
    if (client_.isConnected()) return true;
    std::string ip = this->get_parameter("device_ip").as_string();
    int port = this->get_parameter("device_port").as_int();
    RCLCPP_INFO(this->get_logger(), "重新连接 %s:%d...", ip.c_str(), port);
    return client_.connectServer(ip, static_cast<uint16_t>(port));
  }

  void pollPressure() {
    if (!ensureConnected()) return;
    int16_t val = 0;
    if (!client_.readPressureInt16(val)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "读取气压失败");
      return;
    }
    auto msg = std_msgs::msg::Float32();
    msg.data = static_cast<float>(val);
    pressure_pub_->publish(msg);

    auto info = std_msgs::msg::String();
    info.data = "气压: " + std::to_string(val) + " kPa";
    pressure_info_pub_->publish(info);
  }

  void handleCommand(
      const std::shared_ptr<GripperCommand::Request> request,
      std::shared_ptr<GripperCommand::Response> response) {
    if (!ensureConnected()) {
      response->success = false;
      response->message = "设备未连接";
      return;
    }

    uint8_t slave = request->slave_id;
    uint8_t cmd = request->command;
    response->result_value = 0;

    switch (cmd) {
      // ---- 写操作 ----
      case GripperCmd::START_POSITIVE_PRESSURE: {
        bool ok = client_.startPositivePressure(slave);
        response->success = ok;
        response->message = ok ? "已启动正压" : "启动正压失败";
        break;
      }
      case GripperCmd::RELEASE_POSITIVE_PRESSURE: {
        bool ok = client_.releasePositivePressure(slave);
        response->success = ok;
        response->message = ok ? "已正压松气" : "正压松气失败";
        break;
      }
      case GripperCmd::START_NEGATIVE_PRESSURE: {
        bool ok = client_.startNegativePressure(slave);
        response->success = ok;
        response->message = ok ? "已启动负压" : "启动负压失败";
        break;
      }
      case GripperCmd::RELEASE_NEGATIVE_PRESSURE: {
        bool ok = client_.releaseNegativePressure(slave);
        response->success = ok;
        response->message = ok ? "已负压松气" : "负压松气失败";
        break;
      }
      case GripperCmd::SET_PRESSURE_UPPER_LIMIT: {
        uint16_t val = static_cast<uint16_t>(request->value);
        bool ok = client_.setPressureUpperLimit(val, slave);
        response->success = ok;
        response->message = ok
          ? "已设置气压上限=" + std::to_string(val)
          : "设置气压上限失败";
        break;
      }
      case GripperCmd::SET_NEGATIVE_PRESSURE: {
        int16_t val = static_cast<int16_t>(request->value);
        bool ok = client_.setNegativePressure(val, slave);
        response->success = ok;
        response->message = ok
          ? "已设置负压值=" + std::to_string(val)
          : "设置负压值失败";
        break;
      }
      // ---- 读操作 ----
      case GripperCmd::READ_POSITIVE_FEEDBACK: {
        bool val = false;
        bool ok = client_.readPositiveFeedback(val, slave);
        response->success = ok;
        response->result_value = val ? 1 : 0;
        response->message = ok
          ? "正压反馈=" + std::to_string(val)
          : "读取正压反馈失败";
        break;
      }
      case GripperCmd::READ_NEGATIVE_FEEDBACK: {
        bool val = false;
        bool ok = client_.readNegativeFeedback(val, slave);
        response->success = ok;
        response->result_value = val ? 1 : 0;
        response->message = ok
          ? "负压反馈=" + std::to_string(val)
          : "读取负压反馈失败";
        break;
      }
      default: {
        response->success = false;
        response->message = "未知命令: " + std::to_string(cmd)
          + " (有效值 0~7)";
        break;
      }
    }

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
  }
};

// ============================================================
// 主函数
// ============================================================
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperServer>();

  // 两条执行线程分别服务 /gripper_command 与气压轮询 callback group，
  // ModbusTcpClient 内部用 recursive_mutex 串行化 socket 收发。
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
