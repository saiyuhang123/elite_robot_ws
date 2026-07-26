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
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available, dropping trajectory");
      return;
    }

    auto goal = FollowJT::Goal();
    goal.trajectory = *msg;

    RCLCPP_INFO(this->get_logger(), "Forwarding trajectory with %zu points",
      msg->points.size());

    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleFollowJT::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Trajectory succeeded");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Trajectory failed: %d", (int)result.code);
        }
      };

    action_client_->async_send_goal(goal, send_goal_options);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EliteJointTrajectoryBridge>());
  rclcpp::shutdown();
  return 0;
}
