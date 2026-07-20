/**
 * 抓取运动 MoveIt 服务节点
 *
 * 提供 service ~/move_to_pose (grasp_moveit_msgs/srv/MoveToPose)：
 *   输入：基座系(cs66_base_link)下 cs66_tool0 的目标位姿(位置+姿态) + 速度/加速度缩放
 *   行为：setPoseTarget -> OMPL 关节空间规划 -> execute
 *   输出：success + message（规划/执行失败的原因）
 *
 * 与控制器 movel 不同，MoveIt 在关节空间采样规划，只要求终点可达，
 * 自动绕开奇异位姿，成功率远高于严格直线 movel。
 *
 * 前提：my_elite_robot_cell_moveit_config 的 move_group 已启动。
 * 运行：ros2 run hello_moveit grasp_move_server
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <grasp_moveit_msgs/srv/move_to_pose.hpp>

using MoveToPose = grasp_moveit_msgs::srv::MoveToPose;

class GraspMoveServer : public rclcpp::Node
{
public:
  GraspMoveServer()
  : Node("grasp_move_server",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "elite_arm");
    tip_link_ = declare_parameter<std::string>("tip_link", "cs66_tool0");
    base_frame_ = declare_parameter<std::string>("base_frame", "cs66_base_link");
    planning_time_ = declare_parameter<double>("planning_time", 5.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 5);
    default_vel_scaling_ = declare_parameter<double>("velocity_scaling", 0.2);
    default_acc_scaling_ = declare_parameter<double>("acceleration_scaling", 0.2);

    srv_ = create_service<MoveToPose>(
        "~/move_to_pose",
        std::bind(&GraspMoveServer::onMoveToPose, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

  // MoveGroupInterface 构造会阻塞等待 move_group，须先启动 move_group 节点
  bool initMoveGroup()
  {
    try {
      move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), planning_group_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "连接 move_group 失败: %s", e.what());
      return false;
    }
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setPoseReferenceFrame(base_frame_);
    RCLCPP_INFO(get_logger(),
                "grasp_move_server 就绪: group=%s, tip=%s, base=%s，等待 ~/move_to_pose 请求",
                planning_group_.c_str(), tip_link_.c_str(), base_frame_.c_str());
    return true;
  }

private:
  void onMoveToPose(const std::shared_ptr<MoveToPose::Request> req,
                    std::shared_ptr<MoveToPose::Response> res)
  {
    if (!move_group_) {
      res->success = false;
      res->message = "MoveGroupInterface 未初始化";
      return;
    }

    const double vs = (req->velocity_scaling > 0.0 && req->velocity_scaling <= 1.0)
                          ? req->velocity_scaling : default_vel_scaling_;
    const double as = (req->acceleration_scaling > 0.0 && req->acceleration_scaling <= 1.0)
                          ? req->acceleration_scaling : default_acc_scaling_;
    move_group_->setMaxVelocityScalingFactor(vs);
    move_group_->setMaxAccelerationScalingFactor(as);
    move_group_->setStartStateToCurrentState();

    if (!req->joint_target.empty()) {
      // 关节空间目标（如回零位）
      if (req->joint_target.size() != 6) {
        res->success = false;
        res->message = "joint_target 需要 6 个关节角(rad)，收到 " +
                       std::to_string(req->joint_target.size()) + " 个";
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      std::vector<double> joints(req->joint_target.begin(), req->joint_target.end());
      move_group_->setJointValueTarget(joints);
      RCLCPP_INFO(get_logger(), "收到关节目标: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] (v=%.2f, a=%.2f)",
                  joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], vs, as);
    } else {
      // 位姿目标（base_frame 下 tip_link 的位置+姿态）
      auto target = req->target_pose;
      if (target.header.frame_id.empty()) {
        target.header.frame_id = base_frame_;
      }
      if (target.header.frame_id != base_frame_) {
        res->success = false;
        res->message = "frame_id 必须是 " + base_frame_ + "，收到: " + target.header.frame_id;
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
      move_group_->setPoseTarget(target, tip_link_);
      RCLCPP_INFO(get_logger(), "收到位姿目标: [%.4f, %.4f, %.4f] (v=%.2f, a=%.2f)",
                  target.pose.position.x, target.pose.position.y, target.pose.position.z, vs, as);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_code = move_group_->plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_->clearPoseTargets();
      res->success = false;
      res->message = "MoveIt 规划失败, error_code=" + std::to_string(plan_code.val) +
                     "（多为目标不可达/姿态受限/碰撞）";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "规划成功，开始执行...");
    const auto exec_code = move_group_->execute(plan);
    move_group_->clearPoseTargets();
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS) {
      res->success = false;
      res->message = "MoveIt 执行失败, error_code=" + std::to_string(exec_code.val);
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    res->success = true;
    res->message = "OK";
    RCLCPP_INFO(get_logger(), "执行完成");
  }

  std::string planning_group_, tip_link_, base_frame_;
  double planning_time_, default_vel_scaling_, default_acc_scaling_;
  int planning_attempts_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<MoveToPose>::SharedPtr srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspMoveServer>();
  if (!node->initMoveGroup()) {
    RCLCPP_ERROR(rclcpp::get_logger("grasp_move_server"),
                 "请先启动 move_group: ros2 launch my_elite_robot_cell_moveit_config move_group.launch.py");
    rclcpp::shutdown();
    return 1;
  }
  // 多线程执行器：service 回调里 plan()/execute() 需要其他线程处理 move_group 的响应
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
