/**
 * Elite CS 机械臂运动命令行工具（单发模式：一次运行只发一条轨迹）
 *
 * 两种模式：
 *   cartesian - 笛卡尔轨迹（控制器内置 IK，直线路径；奇异位姿会被拒）
 *   joint     - 用 KDL(LMA) 只求位置逆解（姿态自由、手腕近似保持不动），
 *               再走关节空间轨迹。关节空间运动不经过笛卡尔奇异点。
 *
 * 注意：同一连接里一条轨迹失败后通道会进入错误状态，无法再 START 新轨迹，
 *       所以本工具刻意设计为一次运行只发一条；需要换模式时请重新调用本程序。
 *
 * 编译: g++ -O2 -o cartesian_move cartesian_move.cpp -lelite-cs-series-sdk -lorocos-kdl -ldl -lpthread \
 *       -I/home/nvidia/Documents/elite_robot_ws/build/elite_cs_series_sdk/include -I/usr/include/eigen3
 *
 * 用法:
 *   ./cartesian_move <robot_ip> <x> <y> <z> <rx> <ry> <rz> [speed] [accel] [time] [mode] [j1..j6]
 *     mode: cartesian(默认) | joint
 *     j1..j6: 当前关节角(rad)，joint 模式必填（IK 种子）
 *
 * 示例:
 *   ./cartesian_move 192.168.1.212 0.5 0.2 0.4 -1.36 -0.08 -1.46 0.15 0.3 5.0
 *   ./cartesian_move 192.168.1.212 0.5 0.2 0.4 -1.36 -0.08 -1.46 0.15 0.3 5.0 joint 0.1 -1.2 1.5 -1.0 1.1 0.5
 */

#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/DashboardClient.hpp>
#include <Elite/Log.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <Eigen/Core>

#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cstring>

using namespace ELITE;

static const std::string SCRIPT_FILE = "external_control.script";

// CS66 出厂标定 MDH 参数（由 read_mdh 工具从控制器 30001 端口读取，恒定）
static const vector6d_t MDH_ALPHA = {0, 1.5708, 0, 0, 1.5708, -1.5708};
static const vector6d_t MDH_A = {0, 0, -0.42752, -0.391601, 0, 0};
static const vector6d_t MDH_D = {0.160861, 0, 0, 0.147568, 0.0964976, 0.112116};

/**
 * 按 SDK 插件同款方式建 KDL 链：每关节 = RotX(alpha)*Trans(a,0,d) 固定段 + RotZ 关节
 */
static KDL::Chain buildChain() {
    KDL::Chain chain;
    for (int i = 0; i < 6; i++) {
        KDL::Frame rot(KDL::Rotation(KDL::Vector(1, 0, 0),
                                     KDL::Vector(0, std::cos(MDH_ALPHA[i]), std::sin(MDH_ALPHA[i])),
                                     KDL::Vector(0, -std::sin(MDH_ALPHA[i]), std::cos(MDH_ALPHA[i]))));
        KDL::Frame trans(KDL::Vector(MDH_A[i], 0, MDH_D[i]));
        chain.addSegment(KDL::Segment("L" + std::to_string(i), KDL::Joint(KDL::Joint::None), rot * trans));
        chain.addSegment(KDL::Segment("J" + std::to_string(i), KDL::Joint(KDL::Joint::RotZ)));
    }
    return chain;
}

/**
 * 只求"位置"逆解（姿态权重为 0）：阻尼最小二乘自然贴近种子位形，
 * 手腕三轴基本不动，只有大臂正常摆动。纯计算，不连机器人；
 * 解出后做 FK 校验位置误差 < 1cm。
 */
static bool computeIkJoints(const vector6d_t& target_pose, const vector6d_t& seed_joints, vector6d_t& out_joints) {
    KDL::Chain chain = buildChain();

    // 位置权重 1，姿态权重 0 → 只解位置
    Eigen::Matrix<double, 6, 1> L;
    L << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
    KDL::ChainIkSolverPos_LMA ik(chain, L, 1e-10, 2000);

    KDL::JntArray q_seed(6), q_out(6);
    for (int i = 0; i < 6; i++) {
        q_seed(i) = seed_joints[i];
    }
    KDL::Frame target(KDL::Rotation::Identity(),
                      KDL::Vector(target_pose[0], target_pose[1], target_pose[2]));

    int ret = ik.CartToJnt(q_seed, target, q_out);
    if (ret < 0) {
        std::cerr << "IK 无解(错误码 " << ret << ")：目标位置超出工作空间" << std::endl;
        return false;
    }

    // FK 校验位置误差
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame f;
    fk.JntToCart(q_out, f);
    double err = (f.p - target.p).Norm();
    if (err > 0.01) {
        std::cerr << "IK 未收敛到目标（位置残差 " << err * 1000 << " mm），放弃运动" << std::endl;
        return false;
    }

    std::cout << "IK 成功(位置残差 " << err * 1000 << " mm)，各关节变化量(deg): [";
    for (int i = 0; i < 6; i++) {
        out_joints[i] = q_out(i);
        double delta_deg = (q_out(i) - q_seed(i)) * 180.0 / M_PI;
        std::cout << delta_deg << (i + 1 < 6 ? ", " : "");
    }
    std::cout << "]" << std::endl;
    return true;
}

int main(int argc, const char** argv) {
    if (argc < 8) {
        std::cerr << "用法: " << argv[0]
                  << " <robot_ip> <x> <y> <z> <rx> <ry> <rz> [speed] [accel] [time] [mode] [j1..j6]\n"
                  << "  mode: cartesian(默认) | joint\n"
                  << "  j1..j6: 当前关节角(rad)，joint 模式必填（IK 种子）" << std::endl;
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

    std::string mode = (argc >= 12) ? argv[11] : "cartesian";

    // 确定轨迹点：cartesian 直接用位姿；joint 先求 IK
    vector6d_t traj_point;
    bool cartesian = true;
    if (mode == "joint") {
        if (argc < 18) {
            std::cerr << "ERROR: joint 模式需要 j1..j6 当前关节角(rad)" << std::endl;
            return 1;
        }
        vector6d_t seed_joints;
        for (int i = 0; i < 6; i++) {
            seed_joints[i] = std::stod(argv[12 + i]);
        }
        if (!computeIkJoints(pose, seed_joints, traj_point)) {
            return 1;
        }
        cartesian = false;
    } else {
        traj_point = pose;
    }

    std::cout << "=== Elite CS Move Tool (mode: " << (cartesian ? "cartesian" : "joint") << ") ===" << std::endl;
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

    // 4. 发送唯一一条轨迹
    std::cout << "Starting trajectory..." << std::endl;

    std::promise<TrajectoryMotionResult> done_promise;
    driver->setTrajectoryResultCallback([&](TrajectoryMotionResult result) {
        done_promise.set_value(result);
    });

    if (!driver->writeTrajectoryControlAction(TrajectoryControlAction::START, 1, 200)) {
        std::cerr << "ERROR: Failed to start trajectory" << std::endl;
        return 1;
    }
    if (!driver->writeTrajectoryPoint(traj_point, time, 0, cartesian)) {
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

    TrajectoryMotionResult result = TrajectoryMotionResult::FAILURE;
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        result = future.get();
    }
    std::cout << "Motion result: " << static_cast<int>(result)
              << (result == TrajectoryMotionResult::SUCCESS ? " (SUCCESS)" : " (FAILED)") << std::endl;

    // 6. 结束控制
    std::cout << "Sending idle..." << std::endl;
    driver->writeIdle(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    driver->stopControl();
    dashboard->disconnect();

    std::cout << "=== Done ===" << std::endl;
    return (result == TrajectoryMotionResult::SUCCESS) ? 0 : 1;
}
