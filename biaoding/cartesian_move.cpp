/**
 * Elite CS 机械臂笛卡尔运动命令行工具
 *
 * 编译: g++ -o cartesian_move cartesian_move.cpp -lelite-cs-series-sdk -I/usr/local/include/Elite -L/usr/local/lib
 *       (或者使用 CMake)
 *
 * 用法: ./cartesian_move <robot_ip> <x> <y> <z> <rx> <ry> <rz> [speed] [accel] [time]
 *
 * 示例: ./cartesian_move 192.168.1.212 0.5 0.2 0.4 -1.36 -0.08 -1.46 0.15 0.3 5.0
 */

#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/DashboardClient.hpp>
#include <Elite/Log.hpp>

#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace ELITE;

static const std::string SCRIPT_FILE = "external_control.script";

int main(int argc, const char** argv) {
    if (argc < 8) {
        std::cerr << "用法: " << argv[0]
                  << " <robot_ip> <x> <y> <z> <rx> <ry> <rz> [speed] [accel] [time]"
                  << std::endl;
        std::cerr << "示例: " << argv[0]
                  << " 192.168.1.212 0.5 0.2 0.4 -1.36 -0.08 -1.46 0.15 0.3 5.0"
                  << std::endl;
        return 1;
    }

    std::string robot_ip = argv[1];
    vector6d_t pose;
    pose[0] = std::stod(argv[2]);  // x
    pose[1] = std::stod(argv[3]);  // y
    pose[2] = std::stod(argv[4]);  // z
    pose[3] = std::stod(argv[5]);  // rx (rotation vector)
    pose[4] = std::stod(argv[6]);  // ry
    pose[5] = std::stod(argv[7]);  // rz

    float speed = (argc >= 9)  ? std::stof(argv[8])  : 0.15f;
    float accel = (argc >= 10) ? std::stof(argv[9])  : 0.3f;
    float time  = (argc >= 11) ? std::stof(argv[10]) : 5.0f;

    std::cout << "=== Elite CS Cartesian Move Tool ===" << std::endl;
    std::cout << "Robot IP: " << robot_ip << std::endl;
    std::cout << "Target:   [" << pose[0] << ", " << pose[1] << ", " << pose[2]
              << ", " << pose[3] << ", " << pose[4] << ", " << pose[5] << "]" << std::endl;
    std::cout << "Speed: " << speed << " m/s, Accel: " << accel << " m/s^2, Time: " << time << "s" << std::endl;

    // 1. 配置并连接 Dashboard
    EliteDriverConfig config;
    config.robot_ip = robot_ip;
    config.headless_mode = true;
    config.script_file_path = SCRIPT_FILE;
    // 使用不同于 ROS2 驱动的端口，避免冲突
    // ROS2 驱动默认: reverse=50001, script_sender=50002, trajectory=50003, script_command=50004
    config.reverse_port = 50101;
    config.script_sender_port = 50102;
    config.trajectory_port = 50103;
    config.script_command_port = 50104;

    auto driver = std::make_unique<EliteDriver>(config);
    auto dashboard = std::make_unique<DashboardClient>();

    std::cout << "Connecting to dashboard..." << std::endl;
    if (!dashboard->connect(robot_ip)) {
        std::cerr << "ERROR: Failed to connect to dashboard" << std::endl;
        return 1;
    }
    std::cout << "Dashboard connected." << std::endl;

    // 2. 上电 + 释放抱闸
    std::cout << "Power on..." << std::endl;
    if (!dashboard->powerOn()) {
        std::cerr << "ERROR: Power on failed" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Brake release..." << std::endl;
    if (!dashboard->brakeRelease()) {
        std::cerr << "ERROR: Brake release failed" << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 3. 发送外部控制脚本
    std::cout << "Sending external control script..." << std::endl;
    if (!driver->sendExternalControlScript()) {
        std::cerr << "ERROR: Failed to send external control script" << std::endl;
        return 1;
    }

    std::cout << "Waiting for robot to connect..." << std::endl;
    int wait_count = 0;
    while (!driver->isRobotConnected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_count++;
        if (wait_count > 100) {  // 10 second timeout
            std::cerr << "ERROR: Timeout waiting for robot connection" << std::endl;
            return 1;
        }
    }
    std::cout << "Robot connected!" << std::endl;

    // 4. 发送笛卡尔轨迹点
    std::cout << "Starting Cartesian trajectory..." << std::endl;

    std::promise<TrajectoryMotionResult> done_promise;
    driver->setTrajectoryResultCallback([&](TrajectoryMotionResult result) {
        done_promise.set_value(result);
    });

    if (!driver->writeTrajectoryControlAction(TrajectoryControlAction::START, 1, 200)) {
        std::cerr << "ERROR: Failed to start trajectory" << std::endl;
        return 1;
    }

    // 发送笛卡尔目标点 (is_cartesian=true，机器人内置 IK)
    if (!driver->writeTrajectoryPoint(pose, time, 0, true)) {
        std::cerr << "ERROR: Failed to write trajectory point" << std::endl;
        return 1;
    }

    if (!driver->writeTrajectoryControlAction(TrajectoryControlAction::NOOP, 0, 200)) {
        std::cerr << "ERROR: Failed to send NOOP" << std::endl;
        return 1;
    }

    std::cout << "Trajectory point sent. Waiting for motion to complete..." << std::endl;

    // 5. 等待运动完成（发送 NOOP 保持连接）
    auto future = done_promise.get_future();
    int noop_count = 0;
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (!driver->writeTrajectoryControlAction(TrajectoryControlAction::NOOP, 0, 200)) {
            std::cerr << "ERROR: NOOP failed during motion wait" << std::endl;
            break;
        }
        noop_count++;
        if (noop_count > (time + 10) * 10) {
            std::cerr << "WARN: Motion timeout, sending idle..." << std::endl;
            driver->writeIdle(0);
            break;
        }
    }

    auto result = future.get();
    std::cout << "Motion result: " << static_cast<int>(result);

    if (result == TrajectoryMotionResult::SUCCESS) {
        std::cout << " (SUCCESS)" << std::endl;
    } else {
        std::cout << " (code)" << std::endl;
    }

    // 6. 结束控制
    std::cout << "Sending idle..." << std::endl;
    driver->writeIdle(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    driver->stopControl();
    dashboard->disconnect();

    std::cout << "=== Done ===" << std::endl;
    return (result == TrajectoryMotionResult::SUCCESS) ? 0 : 1;
}
