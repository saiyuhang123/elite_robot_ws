#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

class EliteJointTrajectoryBridge : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;

  EliteJointTrajectoryBridge()
  : Node("elite_joint_trajectory_bridge")
  {
    // Subscribe to the same topic ysrob uses
    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/YsUR_driver/joint_trajectory", 10,
      std::bind(&EliteJointTrajectoryBridge::trajectoryCallback, this, std::placeholders::_1));

    // Action client to Elite's scaled_joint_trajectory_controller
    action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    RCLCPP_INFO(this->get_logger(),
      "Bridge ready: /YsUR_driver/joint_trajectory -> "
      "/scaled_joint_trajectory_controller/follow_joint_trajectory");
  }

private:
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    const bool is_contact_approach = msg->header.frame_id == "contact_approach";

    // 402 接近阶段发布的是一个个必须完整执行的 1 mm 小步。若前一步尚未结束，
    // 把后续消息继续转成 action goal 会不断抢占控制器，使机械臂每次刚起步就减速，
    // 实测 25 s 只能走约 2.4 mm。这里对 contact_approach 做串行化；其他轨迹
    // （特别是接触后的 force_contact_hold）仍可立即发送并抢占当前接近动作。
    if (is_contact_approach && contact_approach_goal_active_) {
      RCLCPP_DEBUG(this->get_logger(),
        "Contact approach step still active; dropping overlapping goal");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available, dropping trajectory");
      return;
    }

    auto goal = FollowJT::Goal();
    goal.trajectory = *msg;

    // 流式打磨每 40ms 一个 goal, 该日志会刷屏, 注释掉(需要时开 DEBUG 级别)
    // RCLCPP_DEBUG(this->get_logger(), "Forwarding trajectory with %zu points",
    //   msg->points.size());

    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    // goal 被拒绝（如 time_from_start 非递增、关节名不匹配）时必须可见，否则表现为"不动且无报错"
    send_goal_options.goal_response_callback =
      [this, is_contact_approach](const GoalHandleFollowJT::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Trajectory goal was REJECTED by controller");
          if (is_contact_approach) {
            contact_approach_goal_active_ = false;
          }
        } else {
          RCLCPP_DEBUG(this->get_logger(), "Trajectory goal accepted");
        }
      };
    send_goal_options.result_callback =
      [this, is_contact_approach](const GoalHandleFollowJT::WrappedResult & result) {
        if (is_contact_approach) {
          contact_approach_goal_active_ = false;
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          if (is_contact_approach) {
            RCLCPP_DEBUG(this->get_logger(), "Contact approach step succeeded");
          } else {
            RCLCPP_INFO(this->get_logger(), "Trajectory succeeded");
          }
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          // 流式打磨时新 goal 抢占旧 goal, 旧 goal 以 CANCELED(code 5)结束, 属正常现象,
          // 每 40ms 刷屏, 直接注释掉
          // RCLCPP_DEBUG(this->get_logger(), "Trajectory preempted (code: %d)", (int)result.code);
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          const int controller_error = result.result ? result.result->error_code : 0;
          const char * error_text = result.result ? result.result->error_string.c_str() : "no result payload";
          RCLCPP_ERROR(this->get_logger(),
            "Trajectory aborted by controller: error_code=%d message=%s", controller_error, error_text);
        } else {
          RCLCPP_WARN(this->get_logger(), "Trajectory finished with code: %d", (int)result.code);
        }
      };

    if (is_contact_approach) {
      // 在 async_send_goal 之前置位，覆盖 goal-response 尚未返回的窗口。
      contact_approach_goal_active_ = true;
    }
    action_client_->async_send_goal(goal, send_goal_options);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  bool contact_approach_goal_active_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EliteJointTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
