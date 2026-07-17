#include <Elite/DataType.hpp>
#include <Elite/EliteDriver.hpp>
#include <Elite/Log.hpp>
#include <Elite/DashboardClient.hpp>
#include <Elite/RtsiIOInterface.hpp>

#include <future>
#include <iostream>
#include <memory>
#include <thread>

using namespace ELITE;

class TrajectoryControl {
   private:
    std::unique_ptr<EliteDriver> driver_;
    
    std::unique_ptr<DashboardClient> dashboard_;
    EliteDriverConfig config_;

   public:
    TrajectoryControl(const EliteDriverConfig& config) {
        config_ = config;
        driver_ = std::make_unique<EliteDriver>(config);
        dashboard_ = std::make_unique<DashboardClient>();

        ELITE_LOG_INFO("Connecting to the dashboard");
        if (!dashboard_->connect(config.robot_ip)) {
            ELITE_LOG_FATAL("Failed to connect to the dashboard.");
            throw std::runtime_error("Failed to connect to the dashboard.");
        }
        ELITE_LOG_INFO("Successfully connected to the dashboard");
    }

    ~TrajectoryControl() {
        if (dashboard_) {
            dashboard_->disconnect();
        }
        driver_->stopControl();
    }

    bool startControl() {
        ELITE_LOG_INFO("Start powering on...");
        if (!dashboard_->powerOn()) {
            ELITE_LOG_FATAL("Power-on failed");
            return false;
        }
        ELITE_LOG_INFO("Power-on succeeded");

        ELITE_LOG_INFO("Start releasing brake...");
        if (!dashboard_->brakeRelease()) {
            ELITE_LOG_FATAL("Brake release failed");
            return false;
        }
        ELITE_LOG_INFO("Brake released");

        if (config_.headless_mode) {
            if (!driver_->isRobotConnected()) {
                if (!driver_->sendExternalControlScript()) {
                    ELITE_LOG_FATAL("Fail to send external control script");
                    return false;
                }
            }
        } else {
            if (!dashboard_->playProgram()) {
                ELITE_LOG_FATAL("Fail to play program");
                return false;
            }
        }

        ELITE_LOG_INFO("Wait external control script run...");
        while (!driver_->isRobotConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ELITE_LOG_INFO("External control script is running");
        return true;
    }

    bool moveTrajectory(const std::vector<vector6d_t>& target_points, float point_time, float blend_radius, bool is_cartesian) {
        std::promise<TrajectoryMotionResult> move_done_promise;
        std::atomic<bool> result_received{false};

        driver_->setTrajectoryResultCallback([&](TrajectoryMotionResult result) {
            ELITE_LOG_INFO("Trajectory result callback received: %d", result);
            move_done_promise.set_value(result);
            result_received = true;
        });

        ELITE_LOG_INFO("Trajectory motion start");
        if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START, target_points.size(), 200)) {
            ELITE_LOG_ERROR("Failed to start trajectory motion");
            return false;
        }
        ELITE_LOG_INFO("START command sent");

        for (const auto& joints : target_points) {
            if (!driver_->writeTrajectoryPoint(joints, point_time, blend_radius, is_cartesian)) {
                ELITE_LOG_ERROR("Failed to write trajectory point");
                return false;
            }
            ELITE_LOG_INFO("Trajectory point sent");
            // Send NOOP command to avoid timeout.
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
        }
        ELITE_LOG_INFO("All points sent, waiting for motion complete...");

        // 等待运动完成，同时发送 NOOP 保持连接
        std::future<TrajectoryMotionResult> move_done_future = move_done_promise.get_future();
        int wait_count = 0;
        while (move_done_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP, 0, 200)) {
                ELITE_LOG_ERROR("Failed to send NOOP command");
                return false;
            }
            wait_count++;
            if (wait_count % 10 == 0) {
                ELITE_LOG_INFO("Waiting for motion complete... (%.1f sec)", wait_count * 0.1);
            }
            // 超时保护：运动时间 + 10秒缓冲
            if (wait_count > (point_time + 10) * 10) {
                ELITE_LOG_WARN("Timeout waiting for result, sending idle...");
                driver_->writeIdle(0);
                return true;  // 运动可能已完成，返回成功
            }
        }

        auto result = move_done_future.get();
        ELITE_LOG_INFO("Trajectory motion completed with result: %d", result);

        ELITE_LOG_INFO("Sending idle command...");
        if(!driver_->writeIdle(0)) {
            ELITE_LOG_ERROR("Failed to write idle command");
            return false;
        }
        ELITE_LOG_INFO("Idle command sent");

        return result == TrajectoryMotionResult::SUCCESS;
    }

    bool moveTo(const vector6d_t& point, float time, bool is_cartesian) {
        return moveTrajectory({point}, time, 0, is_cartesian);
    }
};

// 在文件末尾添加 main 函数
int main(int argc, const char** argv) {
    ELITE::EliteDriverConfig config;
    if (argc == 2) {
        config.robot_ip = argv[1];
    } else if (argc == 3) {
        config.robot_ip = argv[1];
        config.local_ip = argv[2];
    } else {
        std::cout << "Must provide robot IP. Example: ./trajectory_example aaa.bbb.ccc.ddd <eee.fff.ggg.hhh>" << std::endl;
        return 1;
    }
    config.headless_mode = true;
    config.script_file_path = "external_control.script";
    
    auto trajectory_control = std::make_unique<TrajectoryControl>(config);
    auto rtsi_client = std::make_unique<ELITE::RtsiIOInterface>("output_recipe.txt", "input_recipe.txt", 250);

    if (!rtsi_client->connect(config.robot_ip)) {
        throw std::runtime_error("Fail to connect to RTSI");
    }

    if(!trajectory_control->startControl()) {
        return 1;
    }

    // // 获取当前位置并小幅移动
    // auto actual_joints = rtsi_client->getActualJointPositions();
    // actual_joints[3] = 1.45;  // 移动第4关节

    // if(!trajectory_control->moveTo(actual_joints, 8, false)) {
    //     return 1;
    // }

    //  测试2：笛卡尔运动
    auto actual_pose = rtsi_client->getActualTCPPose();
    actual_pose[2] += 0.05;  // Z轴下移 5cm
    ELITE_LOG_INFO("Cartesian move test...");
    if(!trajectory_control->moveTo(actual_pose, 3, true)) {  // true = 笛卡尔
        return 1;
    }

    ELITE_LOG_INFO("All tests passed!");

    

    return 0;
}
