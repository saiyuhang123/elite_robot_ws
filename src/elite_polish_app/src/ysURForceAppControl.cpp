#include "ysURForceAppControl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <algorithm>
#include <cmath>
#include <list>
#include <chrono>
#include <future>
#include <eli_common_interface/srv/set_io.hpp>

namespace elite_robot {
    namespace ur_force_app {
      using namespace std::chrono_literals;
      using std::placeholders::_1;

      ysURForceAppControl::ysURForceAppControl() 
              : Node("ysURForceAppControl") 
      {
          //init app
          app_cmd_ = -1;
          sub_step_ = -1;
          joint_size_ = 6;
          speed_level_ = this->declare_parameter<int>("speed_level", 3);
          initDataQ();

          RCLCPP_INFO(this->get_logger(),"init force data");
          //init force control（参数见 config/polish_params.yaml）
          target_fz_ = this->declare_parameter<double>("target_fz", -8.0);//N
          adjust_dz_ = this->declare_parameter<double>("adjust_dz", 0.0003);//m
          // 2026-08-01: 力控死区(N)。原写死 3N，target -3N 时死区覆盖(-6,0)N，
          // 轻接触不修正导致"不贴合"；1.5N 让修正更主动。
          force_deadband_ = this->declare_parameter<double>("force_deadband", 1.5);
          // 2026-08-01: 402 接触下压每条轨迹的步进(m)。桥接端保证每个 0.48s
          // 小步完整执行；0.001 对应约 2mm/s，避免接触冲击触发碰撞报警。
          contact_step_ = this->declare_parameter<double>("contact_step", 0.001);
          // 2026-08-02: 控制器内建力控(startForceMode)。true=404 不再自己做 ±0.5mm
          // bang-bang 修正，z 向恒力由机器人控制器内部闭环完成。
          use_force_mode_ = this->declare_parameter<bool>("use_force_mode", true);
          // SDK wrench 是机器人施加给环境的力，不是传感器读到的反作用力。
          // 当前 FIX 参考系 z+ 与世界 x+ 重合，因此正值才会向工件逼近。
          force_mode_wrench_z_ = this->declare_parameter<double>("force_mode_wrench_z", 0.9);
          // 力控 z 轴允许的最大调整速度(m/s)，防过冲。
          force_mode_z_vel_limit_ = this->declare_parameter<double>("force_mode_z_vel_limit", 0.004);
          force_mode_sensor_target_fz_ = this->declare_parameter<double>("force_mode_sensor_target_fz", 0.0);
          force_mode_verify_tolerance_ = this->declare_parameter<double>("force_mode_verify_tolerance", 7.0);
          force_mode_verify_time_ = this->declare_parameter<double>("force_mode_verify_time", 0.5);
          force_mode_verify_timeout_ = this->declare_parameter<double>("force_mode_verify_timeout", 5.0);
          force_mode_max_axial_deviation_ = this->declare_parameter<double>("force_mode_max_axial_deviation", 0.040);
          force_mode_monitor_log_period_ = this->declare_parameter<double>("force_mode_monitor_log_period", 1.0);
          force_mode_abort_fz_ = this->declare_parameter<double>("force_mode_abort_fz", -15.0);
          force_mode_hard_abort_fz_ = this->declare_parameter<double>("force_mode_hard_abort_fz", -15.0);
          force_mode_hard_abort_confirm_time_ = this->declare_parameter<double>(
            "force_mode_hard_abort_confirm_time", 0.03);
          force_mode_abort_confirm_time_ = this->declare_parameter<double>(
            "force_mode_abort_confirm_time", 0.12);
          // 2026-08-02: 侧向力保护。两次停摆六维力显示侧向Fx/Fy(10~15N)先于Fz失控，
          // 平均侧向合力持续超阈值即按同一流程退出，比等Fz耦合超限更早、损伤更小。
          force_mode_abort_lateral_f_ = this->declare_parameter<double>("force_mode_abort_lateral_f", 35.0);
          force_mode_lateral_abort_confirm_time_ = this->declare_parameter<double>(
            "force_mode_lateral_abort_confirm_time", 0.12);
          // 2026-08-02: 力控开放Rx/Ry旋转柔顺(目标力矩0)，端面贴合板面卸侧向拖曳力。
          // 旋转柔顺默认关闭: 实机曾出现姿态柔顺振荡和大侧向冲击。
          force_mode_rot_compliance_ = this->declare_parameter<bool>("force_mode_rot_compliance", false);
          force_mode_rot_vel_limit_ = this->declare_parameter<double>("force_mode_rot_vel_limit", 0.05);
          // 2026-08-02: 应用层慢姿态环(替代SDK旋转柔顺)。404用20帧平均力矩慢积分
          // 修正轨迹姿态, 增益小+死区+±2°限幅+0.5°/s限速, 只追准静态贴合误差。
          orient_adapt_enabled_ = this->declare_parameter<bool>("orient_adapt_enabled", false);
          orient_adapt_gain_ = this->declare_parameter<double>("orient_adapt_gain", 0.007);
          orient_adapt_torque_deadband_ = this->declare_parameter<double>("orient_adapt_torque_deadband", 0.10);
          orient_adapt_max_angle_ = this->declare_parameter<double>("orient_adapt_max_angle", 0.035);
          orient_adapt_max_rate_ = this->declare_parameter<double>("orient_adapt_max_rate", 0.009);
          // 2026-08-02: 进给门控。过载暂停推进轨迹索引, 消除尖峰后"压着走"的二次爬升。
          feed_gate_enabled_ = this->declare_parameter<bool>("feed_gate_enabled", true);
          feed_gate_fz_ = this->declare_parameter<double>("feed_gate_fz", -5.5);
          feed_gate_timeout_ = this->declare_parameter<double>("feed_gate_timeout", 3.0);
          // 2026-08-02: 405 退刀前等 endForceMode 响应并静置, 等驱动侧控制器完成切换。
          retract_disable_settle_time_ = this->declare_parameter<double>("retract_disable_settle_time", 1.0);
          force_mode_min_contact_fz_ = this->declare_parameter<double>("force_mode_min_contact_fz", -0.08);
          force_mode_contact_loss_timeout_ = this->declare_parameter<double>("force_mode_contact_loss_timeout", 2.0);
          polish_tool_spinup_time_ = this->declare_parameter<double>("polish_tool_spinup_time", 1.0);
          polish_tool_io_timeout_ = this->declare_parameter<double>("polish_tool_io_timeout", 2.0);
          control_dt_count_ = 10;//todo, n*4ms for timer
          // 调试/工艺参数（ROS 参数，可在 launch 中覆盖）:
          // debug_skip_force_contact=true 时空跑: 402 免接触直接过、404 力控旁路
          // contact_fz_threshold: 402 接触判定阈值(N)，负值，压向工件时 force.z 小于它判定接触
          debug_skip_force_contact_ = this->declare_parameter<bool>("debug_skip_force_contact", false);
          contact_fz_threshold_ = this->declare_parameter<double>("contact_fz_threshold", target_fz_*2);
          // 2026-08-02: 力符号约定。+1=压板读负(旧竖直板); 地面板法兰向下实测压板读正,
          // 取-1翻回"压=负"(用户实测"向法兰内部压为正")。仅符号, 不影响阈值。
          contact_force_sign_ = this->declare_parameter<double>("contact_force_sign", 1.0);
          contact_confirm_samples_ = std::max(
            1, static_cast<int>(this->declare_parameter<int>("contact_confirm_samples", 8)));
          contact_tare_samples_ = std::max(
            1, static_cast<int>(this->declare_parameter<int>("contact_tare_samples", 25)));
          contact_settle_time_ = this->declare_parameter<double>("contact_settle_time", 0.3);
          contact_hold_time_ = this->declare_parameter<double>("contact_hold_time", 0.25);
          contact_hold_completion_margin_ = this->declare_parameter<double>("contact_hold_completion_margin", 0.20);
          contact_joint_velocity_limit_ = this->declare_parameter<double>("contact_joint_velocity_limit", 0.01);
          contact_max_travel_ = this->declare_parameter<double>("contact_max_travel", 0.15);
          contact_timeout_ = this->declare_parameter<double>("contact_timeout", 90.0);
          contact_ik_max_failures_ = std::max(
            1, static_cast<int>(this->declare_parameter<int>("contact_ik_max_failures", 5)));
          debug_approach_time_ = this->declare_parameter<double>("debug_approach_time", 4.0);//空跑接近轨迹时长(s)
          control_dt_index_ = 0;
          //polish data
          frame_polishcloud_base_.p = KDL::Vector(-0.11, 0.125, 0.205);
          frame_polishcloud_base_.M = KDL::Rotation::RPY(M_PI/2, 0, 0);
          frame_polishcloud_transform_.p = KDL::Vector(0,0,0);
          frame_polishcloud_transform_.M = KDL::Rotation::RPY(0, 0, 0);
          frame_forceadjust_base_.p = KDL::Vector(0,0,0);
          frame_forceadjust_base_.M = KDL::Rotation::RPY(0, 0, 0);
          polishcurve_radius_ = this->declare_parameter<double>("polishcurve_radius", 0.9306);
          polishcurve_center_dz_ = this->declare_parameter<double>("polishcurve_center_dz", 1.89539);
          polishcurve_start_ry_ = this->declare_parameter<double>("polishcurve_start_ry_deg", -9.2)*M_PI/180;
          polishcurve_end_ry_ = this->declare_parameter<double>("polishcurve_end_ry_deg", 9.2)*M_PI/180;
          polishproduct_width_ = this->declare_parameter<double>("polishproduct_width", 0.2);
          polishcurve_step_count_ = 500*speed_level_;
          polishcurve_step_index_ = 0;
          polishcurve_ycount_ = this->declare_parameter<int>("polishcurve_ycount", 1);
          polishcurve_yindex_ = 0;
          sidepolish_step_count_ = 100*speed_level_;
          sidepolish_step_index_ = 0;
          dz_polish_startup_tool_ = this->declare_parameter<double>("dz_polish_startup_tool", -0.08);//offset tool z for polish startup
          //退刀参数: 世界系"上"方向(45°倾斜实测) + 抬刀高度
          auto wup = this->declare_parameter<std::vector<double>>("world_up_in_base", {-0.7431, 0.0120, 0.6691});
          world_up_in_base_ = KDL::Vector(wup[0], wup[1], wup[2]);
          retract_lift_height_ = this->declare_parameter<double>("retract_lift_height", 0.15);
          retract_axial_distance_ = this->declare_parameter<double>("retract_axial_distance", 0.03);
          calcCurvePolishPath();

          RCLCPP_INFO(this->get_logger(),"init robot data");
          //init  robot
          joint_eps_ = 1e-2;
          joint_names_.push_back("shoulder_pan_joint");
          joint_names_.push_back("shoulder_lift_joint");
          joint_names_.push_back("elbow_joint");
          joint_names_.push_back("wrist_1_joint");
          joint_names_.push_back("wrist_2_joint");
          joint_names_.push_back("wrist_3_joint");
          ys_prefix_ = "cs66_";

          ys_cur_q_.resize(joint_size_);
          ys_cur_vel.resize(joint_size_);
          for (int i = 0; i < joint_size_; ++i) {
              ys_cur_q_(i) = 0;
              ys_cur_vel(i) = 0;
          }
          ys_first_q_ = true;

          RCLCPP_INFO(this->get_logger(),"init force torque data");
          //ftsensor
          ys_origin_wrench_sensor_.force = KDL::Vector(0,0,0);
          ys_origin_wrench_sensor_.torque = KDL::Vector(0,0,0);
          ys_contact_wrench_sensor_.force = KDL::Vector(0,0,0);
          ys_contact_wrench_sensor_.torque = KDL::Vector(0,0,0);
          ys_contact_wrench_base_.force = KDL::Vector(0,0,0);
          ys_contact_wrench_base_.torque = KDL::Vector(0,0,0);
          // 2026-08-01: 平均窗口 200→20(参数 wrench_average_count)。200 在 125Hz 下滞后 ~1.6s，
          // 接触过冲严重；20 ≈ 0.16s，判定更快且仍平滑。
          ys_contact_wrench_count = this->declare_parameter<int>("wrench_average_count", 20);
          ys_contact_wrench_arr.resize(ys_contact_wrench_count);
          ys_contact_wrench_index = 0;
          ys_average_wrench_.force = KDL::Vector(0,0,0);
          ys_average_wrench_.torque = KDL::Vector(0,0,0);
          ys_first_wrench_ = true;
          // 重力补偿参数（2026-08-01 起为 yaml 参数 tool_gravity/tool_gcenter）:
          // 实测驱动端负载识别只做"当前姿态归零(tare)"，非全姿态重力补偿——
          // 三姿态原始读数 0~14N 随姿态变化。代码内补偿 = 真实工具重力模型
          // (m*g*世界系向下在base系方向)，配合启动 bias 可抵消 tare 残留。
          // 默认值 = 2026-07-27 负载识别: 1.478kg 含打磨机+相机。
          auto get_grav = [this](const std::string& name, std::vector<double> def) {
            return this->declare_parameter<std::vector<double>>(name, def);
          };
          auto grav = get_grav("tool_gravity", {10.7654, -0.1738, -9.6917});
          auto gcen = get_grav("tool_gcenter", {-0.05632, 0.01936, -0.01986});
          ys_tool_gravity_ = KDL::Vector(grav[0], grav[1], grav[2]);
          ys_tool_gcenter_ = KDL::Vector(gcen[0], gcen[1], gcen[2]);
          // Bias will be auto-calibrated from first 200 samples
          ys_bias_wrench_.force = KDL::Vector(0.0, 0.0, 0.0);
          ys_bias_wrench_.torque = KDL::Vector(0.0, 0.0, 0.0);

          //eye
          ys_vision_job_done_ = false;

          //agv
          agv_go_home_done_ = false;
          agv_go_polish_done_ = false;

          //fk ik
          ys_eye_fk_solver_ = NULL;
          ys_ftsensor_fk_solver_ = NULL;
          ys_tcp_fk_solver_ = NULL;
          ys_tcp_tracik_solver_ = NULL;

          //robot model
          RCLCPP_INFO(this->get_logger(),"init ik fk data");
          urdf::Model robot_model;
          std::string xml_string;
          std::string urdf_param;
          urdf_param = "robot_description";
          auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");
          while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
              rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "/robot_state_publisher node param service not available, waiting again...");
          }
          xml_string = parameters_client->get_parameter<std::string>(urdf_param);
          // RCLCPP_INFO(this->get_logger(),"xml_string: %s", xml_string.c_str());

          if ( !xml_string.empty()) {
              robot_model.initString(xml_string);

              //limit
              ys_min_jnt_.resize(joint_size_);
              ys_max_jnt_.resize(joint_size_);
              ys_vel_limit_.resize(joint_size_);
              //
              urdf::JointConstSharedPtr joint;
              for(int i=0;i<joint_size_;++i)
              {
                joint = robot_model.getJoint(ys_prefix_+joint_names_[i]);
                ys_min_jnt_(i) = joint->limits->lower;
                ys_max_jnt_(i) = joint->limits->upper;
                ys_vel_limit_(i) = joint->limits->velocity;
              }

            KDL::Tree kdl_tree;
            if (kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
              //ys_eye
              // 眼在手上相机: 链只到 tool0（URDF 中无 tr_camera link），
              // tool0->相机光学系的固定变换（手眼标定）在使用处单独乘，见 ysCamera3DSolver.cpp。
              // 此处 FK 仅用于 timer_callback 的守卫判断，链到 tool0 即可。
                std::string ys_base = ys_prefix_ + "base_link";
                std::string ys_tip_link = ys_prefix_ + "tool0";
                if (kdl_tree.getChain(ys_base, ys_tip_link, ys_eye_chain_)) {
                  ys_eye_fk_solver_ = new KDL::ChainFkSolverPos_recursive(ys_eye_chain_);
                } else {
                  RCLCPP_FATAL(this->get_logger(),"Couldn't find chain %s to %s", ys_base.c_str(), ys_tip_link.c_str());
                }
              //ys_ftsensor
                ys_base = ys_prefix_ + "base_link";
                ys_tip_link = ys_prefix_ + "ft_frame";
                if (kdl_tree.getChain(ys_base, ys_tip_link, ys_ftsensor_chain_)) {
                  ys_ftsensor_fk_solver_ = new KDL::ChainFkSolverPos_recursive(ys_ftsensor_chain_);
                } else {
                  RCLCPP_FATAL(this->get_logger(),"Couldn't find chain %s to %s", ys_base.c_str(), ys_tip_link.c_str());
                }
              //ys_tcp
                ys_base = ys_prefix_ + "base_link";
                ys_tip_link = ys_prefix_ + "tool0";
                if (kdl_tree.getChain(ys_base, ys_tip_link, ys_tcp_chain_)) {
                  ys_tcp_fk_solver_ = new KDL::ChainFkSolverPos_recursive(ys_tcp_chain_);
                } else {
                  RCLCPP_FATAL(this->get_logger(),"Couldn't find chain %s to %s", ys_base.c_str(), ys_tip_link.c_str());
                }
                //ik tcp
                double timeout=0.005;
                double joint_eps = 1e-5;
                ys_tcp_tracik_solver_ = new TRAC_IK::TRAC_IK(ys_base, ys_tip_link, xml_string, timeout, joint_eps);
            } else {
                RCLCPP_FATAL(this->get_logger(),"Failed to extract kdl tree from xml robot description");
            }
          }

            RCLCPP_INFO(this->get_logger(),"init publisher and subscriber data");
            //app cmd
            cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(       
                "/elite_forceapp_cmd", 1, std::bind(&ysURForceAppControl::subCommandStateCB, this, _1)); 
            cmd_result_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/elite_forceapp_cmd_result", 1);
            polish_result_detail_publisher_ = this->create_publisher<std_msgs::msg::String>(
                "/elite_forceapp_result_detail", 10);
            //vision
            vision_job_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/elite_vision_job_cmd", 1);
            vision_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(       
                "/elite_vision_pose_broadcaster/pose", 1, std::bind(&ysURForceAppControl::ys_subVisionResultCB, this, _1)); 
            //agv
            agv_cmd_publisher_ = this->create_publisher<std_msgs::msg::String>("ysrob_agv_job_cmd", 1); 
            agv_status_sub_ = this->create_subscription<std_msgs::msg::String>(       
                "/ysrob_agv_job_result", 1, std::bind(&ysURForceAppControl::ys_subAGVJobResultCB, this, _1)); 
            //ur
            ys_jointstates_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(       
                "/joint_states", 1, std::bind(&ysURForceAppControl::ys_subJointStateCB, this, _1)); 
            ys_traj_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/YsUR_driver/joint_trajectory", 1); 
            //ftsensor
            ys_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(       
                "/force_torque_sensor_broadcaster/wrench", 1, std::bind(&ysURForceAppControl::ys_subWrenchCB, this, _1)); 
            ys_contact_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ys_contact_fts_broadcaster/wrench", 1); 
            // 控制器内建力控服务（驱动硬件接口暴露）
            force_mode_client_ = this->create_client<eli_common_interface::srv::ForceMode>("/force_mode_server/set_force_mode");
            // 打磨头控制：对应手动命令
            // ros2 service call /io_and_status_controller/set_io
            //   eli_common_interface/srv/SetIO "{fun: 2,pin: 7,state: true/false}"
            polish_tool_io_client_ = this->create_client<eli_common_interface::srv::SetIO>(
              "/io_and_status_controller/set_io");
            //timer
            motion_timer_ = this->create_wall_timer(
                4ms, std::bind(&ysURForceAppControl::timer_callback, this));
      }

      void ysURForceAppControl::resetPolishOutcome() {
        polish_failed_ = false;
        polish_cancelled_ = false;
        polish_result_detail_.clear();
      }

      void ysURForceAppControl::markPolishFailure(const std::string &reason) {
        if (polish_cancelled_) {
          return;
        }
        if (polish_failed_) {
          return;
        }
        polish_failed_ = true;
        polish_result_detail_ = reason;
        RCLCPP_ERROR(this->get_logger(), "polish result marked FAILED: %s", reason.c_str());
      }

      void ysURForceAppControl::abortPolishing(
          const std::string &reason, bool safe_retract) {
        markPolishFailure(reason);
        ysPolishTool_Close();
        disableForceMode();
        if (safe_retract) {
          publishCurrentPositionHold(contact_hold_time_);
          sub_step_ = 405;
        } else {
          app_cmd_ = AppCommand::GO_HOME;
          sub_step_ = 0;
        }
      }

      void ysURForceAppControl::cancelPolishing(const std::string &reason) {
        polish_cancelled_ = true;
        polish_failed_ = false;
        polish_result_detail_ = reason;
        RCLCPP_WARN(this->get_logger(), "polish result marked CANCELED: %s", reason.c_str());
        ysPolishTool_Close();
        disableForceMode();

        // 402 起机械臂可能已经接近/接触工件，必须走 405→406 的安全退刀；
        // 视觉和预备阶段尚未接触，可直接回 Home2。
        if (app_cmd_ == AppCommand::DO_CURVE_POLISHING
            && sub_step_ >= 402 && sub_step_ <= 404) {
          publishCurrentPositionHold(contact_hold_time_);
          sub_step_ = 405;
        } else if (app_cmd_ == AppCommand::DO_CURVE_POLISHING
                   && sub_step_ >= 405 && sub_step_ <= 407) {
          // 已在安全收尾路径上，不把步骤倒回去。
        } else {
          app_cmd_ = AppCommand::GO_HOME;
          sub_step_ = 0;
        }
      }

      void ysURForceAppControl::publishPolishResult(
          int32_t code, const std::string &detail) {
        std_msgs::msg::String detail_msg;
        detail_msg.data = detail;
        polish_result_detail_publisher_->publish(detail_msg);
        std_msgs::msg::Int32 result_msg;
        result_msg.data = code;
        cmd_result_publisher_->publish(result_msg);
        RCLCPP_INFO(this->get_logger(),
          "pub polish result: code=%d detail=%s", code, detail.c_str());
      }

      bool ysURForceAppControl::initDataQ(){
          // 示教姿态从参数加载（config/polish_params.yaml）。
          // 注意单位: home 为角度制，cameraCapture/polishBase 为弧度制（与示教来源一致）。
          auto home_deg = this->declare_parameter<std::vector<double>>(
              "home_q_deg", {-2.2001579333, 19.6982889966, -154.8017370884,
                              -86.2989031026, 94.1025882723, 84.2018775724});
          ys_home_q_.resize(joint_size_);
          for (int i=0;i<joint_size_;i++){
            ys_home_q_(i) = home_deg[i]*M_PI/180;
          }
          auto cap_rad = this->declare_parameter<std::vector<double>>(
              "camera_capture_q_rad", {0.1204, -0.9960, -2.4689, 0.4922, 1.6232, 1.6037});
          ys_cameraCapture_q_.resize(joint_size_);
          for (int i=0;i<joint_size_;i++){
            ys_cameraCapture_q_(i) = cap_rad[i];
          }
          auto pb_rad = this->declare_parameter<std::vector<double>>(
              "polish_base_q_rad", {0.2583, -1.7438, -1.5621, 0.7505, 1.5043, 1.5330});
          ys_polishBase_q_.resize(joint_size_);
          for (int i=0;i<joint_size_;i++){
            ys_polishBase_q_(i) = pb_rad[i];
          }
          return true;
      }

      void ysURForceAppControl::subCommandStateCB(const std_msgs::msg::Int32 msg) {
        int32_t cmd;
        cmd = msg.data;
        switch (cmd)
        {
        case 0:
          if (app_cmd_ == AppCommand::DO_CAMERA_VISION_JOB
              || app_cmd_ == AppCommand::DO_CURVE_POLISHING) {
            cancelPolishing("收到命令0，取消当前打磨并回 Home2");
          } else {
            resetPolishOutcome();
            app_cmd_ = AppCommand::GO_HOME;
            sub_step_ = app_cmd_*100;
          }
          break;
        case 1:
          app_cmd_ = AppCommand::DO_AGV_GO_HOME;
          sub_step_ = app_cmd_*100;
          break;
        case 2:
          app_cmd_ = AppCommand::DO_AGV_GO_POLISH;
          sub_step_ = app_cmd_*100;
          break;
        case 3:
          // 2026-08-02: 视觉+打磨命令也必须复位打磨状态。原只命令4复位，
          // 导致第二次命令3沿用上次残留: 跳过402重新去皮(旧零点)、切向计时器
          // 显示上次的elapsed、watchdog过力确认计时器残留致一过线即瞬间停摆。
          disableForceMode();
          resetForceContactState();
          resetPolishOutcome();
          app_cmd_ = AppCommand::DO_CAMERA_VISION_JOB;
          sub_step_ = app_cmd_*100;
          break;
        case 4:
          disableForceMode();
          resetForceContactState();
          resetPolishOutcome();
          app_cmd_ = AppCommand::DO_CURVE_POLISHING;
          sub_step_ = app_cmd_*100;
          break;
        case 5:
          if (app_cmd_ == AppCommand::DO_CAMERA_VISION_JOB
              || app_cmd_ == AppCommand::DO_CURVE_POLISHING) {
            cancelPolishing("收到命令5，安全取消打磨");
          } else {
            RCLCPP_WARN(this->get_logger(), "cancel ignored: no polishing job active");
          }
          break;
        case 51:
          ysPolishTool_Close();
          break;
        case 52:
          ysPolishTool_Open();
          break;
        default:
          app_cmd_ = AppCommand::NOTHING;
          sub_step_ = 0;
          break;
        }
        RCLCPP_INFO(this->get_logger(), "subCommandStateCB: %d. : %d", app_cmd_, sub_step_);
      }

      void ysURForceAppControl::timer_callback()                                                       
      {
        if (ys_eye_fk_solver_ 
          &&ys_ftsensor_fk_solver_ 
          &&ys_tcp_fk_solver_ 
          &&ys_tcp_tracik_solver_ 
          ) 
          {
            switch (app_cmd_)
            {
            case AppCommand::GO_HOME:
            if ( ys_first_q_ == false 
            ) {
              goHome();
            }
              break;
            case AppCommand::DO_AGV_GO_HOME:
              doAgvGoHome();
              break;
            case AppCommand::DO_AGV_GO_POLISH:
              doAgvGoPolish();
              break;
            case AppCommand::DO_CAMERA_VISION_JOB:
              if ( ys_first_q_ == false 
              ) {
                doVisionJob();
              }
              break;
            case AppCommand::DO_CURVE_POLISHING:
              if ( ys_first_wrench_ == false 
                  &&ys_first_q_ == false 
              ) {
                doForcePolishing();
              }
              break;
            default:
              break;
            }

          }
      }


      void ysURForceAppControl::ys_subJointStateCB(const sensor_msgs::msg::JointState state) {
          // RCLCPP_INFO(this->get_logger(),"sub ys ur state, %f, %f, %f, %f, %f, %f", 
            // state.position[0]*180/M_PI,state.position[1]*180/M_PI,state.position[2]*180/M_PI,state.position[3]*180/M_PI,state.position[4]*180/M_PI, state.position[5]*180/M_PI);
          //get joint array
          KDL::JntArray jntArr;
          KDL::JntArray jntSpeed;
          jntArr.resize(joint_size_);
          jntSpeed.resize(joint_size_);
          int n = state.name.size();
          for (int i = 0; i < joint_size_; ++i)//joint_names_
          {
              int x = 0;
              for (; x < n; ++x)//state
              {
                  if (state.name[x] == (ys_prefix_ + joint_names_[i])) {
                      jntArr(i) = state.position[x];
                      jntSpeed(i) = state.velocity[x];
                      break;
                  }
              }

              if (x == n) {
                  return;
              }
          }
          ys_cur_q_ = jntArr;
          ys_cur_vel = jntSpeed;
          //fk
          if (ys_tcp_fk_solver_) {
            ys_tcp_fk_solver_->JntToCart(ys_cur_q_, ys_curP_tcp_);
            // RCLCPP_INFO(this->get_logger(),"sub ys ys_curP_tcp_, x: %f, y: %f, z: %f", 
            //   ys_curP_tcp_.p.data[0], ys_curP_tcp_.p.data[1], ys_curP_tcp_.p.data[2]);
            ys_first_q_ = false;
          }
      }

      void ysURForceAppControl::ys_subVisionResultCB(const geometry_msgs::msg::PoseStamped result) {
        std::string id=result.header.frame_id;
        if (id=="failed")
        {
          if (app_cmd_ == AppCommand::DO_CAMERA_VISION_JOB && sub_step_ == 303) {
            abortPolishing("深度视觉定位失败", false);
          } else {
            RCLCPP_WARN(this->get_logger(),
              "ignored stale vision failure (app_cmd_=%d, sub_step_=%d)",
              app_cmd_, sub_step_);
          }
          return;
        }
        
        RCLCPP_INFO(this->get_logger(), "subVisionResultCB: type: %s: x %f ; y %f ; z %f", id.c_str(), result.pose.position.x, result.pose.position.y, result.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "subVisionResultCB: Quaternion: x %f ; y %f ; z %f; w %f", result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z, result.pose.orientation.w);
        if ( (sub_step_ == 303 && ys_vision_job_done_ == false)
          || app_cmd_ == AppCommand::NOTHING || app_cmd_ < 0 )  // 调试: 空闲(-1或NOTHING)时允许手动注入参考系
        {
          frame_polishcloud_transform_.p = KDL::Vector(result.pose.position.x,result.pose.position.y,result.pose.position.z);
          frame_polishcloud_transform_.M = KDL::Rotation::Quaternion(result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z, result.pose.orientation.w);          
          RCLCPP_INFO(this->get_logger(),"subVisionResultCB: frame APPLIED (app_cmd_=%d, sub_step_=%d)", app_cmd_, sub_step_);
          ys_vision_job_done_ = true;
        }
      }

      void ysURForceAppControl::ys_subAGVJobResultCB(const std_msgs::msg::String result) {
        std::string status=result.data;
        RCLCPP_INFO(this->get_logger(),"get agv job result, status: %s", status);
        if (sub_step_ == 101 && status == "GoHomeDone") {
          agv_go_home_done_ = true;
          agv_go_polish_done_ = false;
          RCLCPP_INFO(this->get_logger(),"agv status changed: GoHomeDone");
        } else if (sub_step_ == 201 && status == "GoPolishDone") {
          agv_go_home_done_ = false;
          agv_go_polish_done_ = true;
          RCLCPP_INFO(this->get_logger(),"agv status changed: GoPolishDone");
        } else {
          agv_go_home_done_ = false;
          agv_go_polish_done_ = false;
        }
      }

      void ysURForceAppControl::ys_subWrenchCB(const geometry_msgs::msg::WrenchStamped msg) {
          // RCLCPP_INFO(this->get_logger(),"sub ys wrench: force, x: %f, y: %f, z: %f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
          KDL::Wrench data,sum;
          data.force.data[0] = msg.wrench.force.x;
          data.force.data[1] = msg.wrench.force.y;
          data.force.data[2] = msg.wrench.force.z;
          data.torque.data[0] = msg.wrench.torque.x;
          data.torque.data[1] = msg.wrench.torque.y;
          data.torque.data[2] = msg.wrench.torque.z;

          //origin
          ys_origin_wrench_sensor_ = data;

          //fix bias data
          if (ys_first_q_ == false
            && ys_first_wrench_ == true
            && ys_ftsensor_fk_solver_
          ) {
            KDL::Wrench biasdata = ys_gravityRepairWrench(data);
            ys_contact_wrench_arr[ys_contact_wrench_index] = biasdata;
            ys_contact_wrench_index++;

            if (ys_contact_wrench_index == ys_contact_wrench_count)
            {
              sum.force = KDL::Vector(0,0,0);
              sum.torque = KDL::Vector(0,0,0);
              for (size_t i = 0; i < ys_contact_wrench_count; i++)
              {
                sum += ys_contact_wrench_arr[i];
              }
              RCLCPP_INFO(this->get_logger(),"init bias wrench: force, x: %f, y: %f, z: %f", ys_bias_wrench_.force.data[0], ys_bias_wrench_.force.data[1], ys_bias_wrench_.force.data[2]);
              ys_bias_wrench_ = sum / ys_contact_wrench_count;
              RCLCPP_INFO(this->get_logger(),"device bias wrench: force, x: %f, y: %f, z: %f", ys_bias_wrench_.force.data[0], ys_bias_wrench_.force.data[1], ys_bias_wrench_.force.data[2]);

              ys_contact_wrench_index = 0;
              ys_first_wrench_ = false;

              //set contact wrench arr
              if (ys_ftsensor_fk_solver_) {
                ys_ftsensor_fk_solver_->JntToCart(ys_cur_q_, ys_curP_ftsensor_);
                for (size_t i = 0; i < ys_contact_wrench_count; i++)
                {
                  ys_contact_wrench_arr[i].force = ys_curP_ftsensor_.M*(biasdata-ys_bias_wrench_).force;
                  ys_contact_wrench_arr[i].torque = ys_curP_ftsensor_.M*(biasdata-ys_bias_wrench_).torque;
                }
              }
            }
          }

          //calc contact wrench data
          if (ys_first_wrench_==false) {
            ys_contact_wrench_sensor_ = ys_gravityRepairWrench(data) - ys_bias_wrench_;

            // 保留最近 24 帧相对力，供过力退出时打印诊断序列，定位尖峰形态与触发时机。
            force_history_.push_back(relFz());
            while (force_history_.size() > 24) {
              force_history_.pop_front();
            }

            // 预备姿态保持静止后重新取局部零点。这样可消除启动姿态到打磨姿态之间
            // 的重力模型残差/零漂，避免尚未碰板时单帧约 -2N 被误判为接触。
            if (contact_tare_collecting_ && maxJointSpeed() <= contact_joint_velocity_limit_) {
              contact_tare_sum_ += ys_contact_wrench_sensor_.force.data[2];
              contact_tare_sum_fx_ += ys_contact_wrench_sensor_.force.data[0];
              contact_tare_sum_fy_ += ys_contact_wrench_sensor_.force.data[1];
              contact_tare_count_++;
              if (contact_tare_count_ >= contact_tare_samples_) {
                contact_fz_zero_ = contact_tare_sum_ / static_cast<double>(contact_tare_count_);
                contact_fx_zero_ = contact_tare_sum_fx_ / static_cast<double>(contact_tare_count_);
                contact_fy_zero_ = contact_tare_sum_fy_ / static_cast<double>(contact_tare_count_);
                contact_tare_collecting_ = false;
                RCLCPP_INFO(this->get_logger(),
                  "contact local tare ready: raw_fz_zero=%.3f N raw_fx_zero=%.3f N raw_fy_zero=%.3f N samples=%d",
                  contact_fz_zero_, contact_fx_zero_, contact_fy_zero_, contact_tare_count_);
              }
            }

            // 接触只在新的传感器帧上计数；连续若干帧超过阈值才确认，拒绝瞬时尖峰。
            if (contact_detection_enabled_ && !contact_confirmed_) {
              const double relative_fz = relFz();
              if (relative_fz < contact_fz_threshold_) {
                contact_confirm_count_++;
                if (contact_confirm_count_ >= contact_confirm_samples_) {
                  contact_confirmed_ = true;
                  contact_detection_enabled_ = false;
                }
              } else {
                contact_confirm_count_ = 0;
              }
            }
            if (ys_ftsensor_fk_solver_) {
              ys_ftsensor_fk_solver_->JntToCart(ys_cur_q_, ys_curP_ftsensor_);
              ys_contact_wrench_base_.force = ys_curP_ftsensor_.M*ys_contact_wrench_sensor_.force;
              ys_contact_wrench_base_.torque = ys_curP_ftsensor_.M*ys_contact_wrench_sensor_.torque;

              ys_contact_wrench_arr[ys_contact_wrench_index] = ys_contact_wrench_sensor_;
              ys_contact_wrench_index++;
              if (ys_contact_wrench_count == ys_contact_wrench_index) ys_contact_wrench_index = 0;
              //
              sum.force = KDL::Vector(0,0,0);
              sum.torque = KDL::Vector(0,0,0);
              for (size_t i = 0; i < ys_contact_wrench_count; i++)
              {
                sum += ys_contact_wrench_arr[i];
              }
              ys_average_wrench_ = sum / ys_contact_wrench_count;

              // ys contact wrench pub
              geometry_msgs::msg::WrenchStamped msg_contact_fts;
              msg_contact_fts.header.stamp = this->now();
              msg_contact_fts.header.frame_id = "cs66_base_link";
              msg_contact_fts.wrench.force.x = ys_contact_wrench_sensor_.force.data[0];
              msg_contact_fts.wrench.force.y = ys_contact_wrench_sensor_.force.data[1];
              msg_contact_fts.wrench.force.z = ys_contact_wrench_sensor_.force.data[2];
              msg_contact_fts.wrench.torque.x = ys_contact_wrench_sensor_.torque.data[0];
              msg_contact_fts.wrench.torque.y = ys_contact_wrench_sensor_.torque.data[1];
              msg_contact_fts.wrench.torque.z = ys_contact_wrench_sensor_.torque.data[2];

              ys_contact_wrench_publisher_->publish(msg_contact_fts);   
            }
          }
      }

      KDL::Wrench ysURForceAppControl::ys_gravityRepairWrench(const KDL::Wrench &data) {
        KDL::Wrench value;
        //fk
        if (ys_ftsensor_fk_solver_) {
          ys_ftsensor_fk_solver_->JntToCart(ys_cur_q_, ys_curP_ftsensor_);
          KDL::Vector gravity_sensor = ys_curP_ftsensor_.M.Inverse()*ys_tool_gravity_;
          value.force = data.force - gravity_sensor;
          value.torque.data[0] = data.torque.data[0] 
              - (gravity_sensor.data[2]*ys_tool_gcenter_.data[1] - gravity_sensor.data[1]*ys_tool_gcenter_.data[2]);
          value.torque.data[1] = data.torque.data[1] 
              - (gravity_sensor.data[0]*ys_tool_gcenter_.data[2] - gravity_sensor.data[2]*ys_tool_gcenter_.data[0]);
          value.torque.data[2] = data.torque.data[2] 
              - (gravity_sensor.data[1]*ys_tool_gcenter_.data[0] - gravity_sensor.data[0]*ys_tool_gcenter_.data[1]);
        }
        return value;
      }

      void ysURForceAppControl::calcCurvePolishPath(){
        polishcurve_OriginFrames_.clear();
        KDL::Frame dirFrame;
        dirFrame.p = KDL::Vector(0,0,0);
        dirFrame.M = KDL::Rotation::RPY(0, 0, M_PI);

        size_t yindex=0, stepindex=0, sideindex=0;
        KDL::Frame target;
        for (yindex = 0; yindex < polishcurve_ycount_; yindex++)
        {
          for (stepindex = 0; stepindex < polishcurve_step_count_; stepindex++)
          {
            target = calcCurvePolishPoint(stepindex, yindex);
            polishcurve_OriginFrames_.push_back(target*dirFrame);
          }
          
          if (yindex == (polishcurve_ycount_-1))
          {
            target = calcSidePolishPoint(0, yindex);
            polishcurve_OriginFrames_.push_back(target*dirFrame);
          } else {
            for (sideindex = 0; sideindex < sidepolish_step_count_; sideindex++)
            {
              target = calcSidePolishPoint(sideindex, yindex);
              polishcurve_OriginFrames_.push_back(target*dirFrame);
            }
          }
        }
        RCLCPP_INFO(this->get_logger(),"calcCurv-ePolishPath: point count: %d", polishcurve_OriginFrames_.size());
      }

      KDL::Frame ysURForceAppControl::calcCurvePolishPoint(int stepindex, int yindex){
        // 2026-08-01 直线打磨: 工件板面为平面，扫掠改为固定高度直线。
        // 起止点 x = R*sin(π+ry) 在 start/end_ry 处的值（与圆弧版扫掠端点一致，
        // 扫掠长度仍由 polishcurve_start/end_ry_deg 控制）；z 恒为 center_dz - R；
        // 姿态恒为 RPY(0,0,0)（tool z 始终沿平面法向，不再随圆弧旋转）。
        KDL::Frame target;
        double start_x = polishcurve_radius_ * sin(M_PI + polishcurve_start_ry_);
        double end_x   = polishcurve_radius_ * sin(M_PI + polishcurve_end_ry_);
        if (1 == yindex%2) {  // 之字形: 偶数道 start→end, 奇数道 end→start
          double tmp = start_x; start_x = end_x; end_x = tmp;
        }
        target.p.data[0] = start_x + stepindex*(end_x - start_x) / polishcurve_step_count_;
        target.p.data[1] = polishproduct_width_/2-(0.5+yindex)*(polishproduct_width_/polishcurve_ycount_);
        target.p.data[2] = polishcurve_center_dz_ + polishcurve_radius_ * cos(M_PI);  // = center_dz - R
        target.M = KDL::Rotation::RPY(0, 0, 0);
        // RCLCPP_INFO(this->get_logger(), "Y %d polishcurve %d: x %f ; y %f ; z %f",yindex, stepindex, target.p.data[0], target.p.data[1], target.p.data[2]);

        return target;
      }
      KDL::Frame ysURForceAppControl::calcSidePolishPoint(int sideindex, int yindex){
        // 换道段: x 保持在当前道扫掠末端不动，y 从当前道平移到下一道，z/姿态恒定。
        KDL::Frame target;
        double end_x = polishcurve_radius_ * sin(M_PI + polishcurve_end_ry_);
        if (1 == yindex%2) {  // 奇数道结束在 start_x 端
          end_x = polishcurve_radius_ * sin(M_PI + polishcurve_start_ry_);
        }
        target.p.data[0] = end_x;
        target.p.data[1] = polishproduct_width_/2-(0.5+yindex+(1.0*sideindex/sidepolish_step_count_))*(polishproduct_width_/polishcurve_ycount_);
        target.p.data[2] = polishcurve_center_dz_ + polishcurve_radius_ * cos(M_PI);  // = center_dz - R
        target.M = KDL::Rotation::RPY(0, 0, 0);
        // RCLCPP_INFO(this->get_logger(), "Y %d polishside %d: x %f ; y %f ; z %f", yindex, sideindex, target.p.data[0], target.p.data[1], target.p.data[2]);

        return target;
      }

      void ysURForceAppControl::doAgvGoHome() {
        switch (sub_step_)
        {
        case 100://AppCommand::DO_AGV_GO_HOME*100
          agv_goHomeCommand();
          break;
        case 101:
          agv_WaitGoHomeDone();
          break;
        default:
          break;
        }
      }
      void ysURForceAppControl::agv_goHomeCommand() {
        // Elite: no AGV, skip immediately
        if (sub_step_ == 100) {
          RCLCPP_INFO(this->get_logger(), "AGV GoHome skipped (no AGV on Elite)");
          sub_step_++;
        }
      }
      void ysURForceAppControl::agv_WaitGoHomeDone() {
        // Elite: no AGV, immediately done
        if (sub_step_ == 101) {
          RCLCPP_INFO(this->get_logger(), "AGV GoHome done (skipped)");
          sub_step_ = 9999;
          std_msgs::msg::Int32 msg;
          msg.data = 101;
          cmd_result_publisher_->publish(msg);
          app_cmd_ = AppCommand::NOTHING;
          RCLCPP_INFO(this->get_logger(), "pub the result of command: %d.", msg.data);
        }
      }

      void ysURForceAppControl::doAgvGoPolish() {
        switch (sub_step_)
        {
        case 200://AppCommand::DO_AGV_GO_Polish*100
          agv_goPolishCommand();
          break;
        case 201:
          agv_WaitGoPolishDone();
          break;
        default:
          break;
        }

      }
      void ysURForceAppControl::agv_goPolishCommand() {
        // Elite: no AGV, skip immediately
        if (sub_step_ == 200) {
          RCLCPP_INFO(this->get_logger(), "AGV GoPolish skipped (no AGV on Elite)");
          sub_step_++;
        }
      }
      void ysURForceAppControl::agv_WaitGoPolishDone() {
        // Elite: no AGV, immediately chain to camera vision
        if (sub_step_ == 201) {
          RCLCPP_INFO(this->get_logger(), "AGV GoPolish done (skipped), chaining to vision");
          sub_step_ = 9999;
          std_msgs::msg::Int32 msg;
          msg.data = 102;
          cmd_result_publisher_->publish(msg);
          app_cmd_ = AppCommand::DO_CAMERA_VISION_JOB;
          sub_step_ = app_cmd_ * 100;
          RCLCPP_INFO(this->get_logger(), "pub the result of command: %d.", msg.data);
        }
      }

      void ysURForceAppControl::doVisionJob() {
        switch (sub_step_)
        {
        case 300://AppCommand::DO_CAMERA_VISION_JOB*100
          vision_goCaptureMove();
          break;
        case 301:
          vision_WaitCaptureMoveDone();
          break;
        case 302:
          vision_SendVisionCmd();
          break;
        case 303:
          vision_WaitVisionSolverDone();
          break;
        default:
          break;
        }

      }
      void ysURForceAppControl::vision_goCaptureMove() {
        if (300==sub_step_) {
          // trajectory
          trajectory_msgs::msg::JointTrajectory traj_goal;
          traj_goal.header.stamp = this->now();
          traj_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              traj_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          
          rclcpp::Duration moveT(2*speed_level_, 1E9/125);
          //base point
          trajectory_msgs::msg::JointTrajectoryPoint polish_base_point;
          for(int i=0;i<joint_size_;++i)
          {
            polish_base_point.positions.push_back(ys_cameraCapture_q_(i));
            polish_base_point.velocities.push_back(0);
          }
          polish_base_point.time_from_start = moveT;
          traj_goal.points.push_back(polish_base_point);

          //pub
          RCLCPP_INFO(this->get_logger(),"pub go vision capture trajectory");
          ys_traj_publisher_->publish(traj_goal);   

          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_++;
        }
      }
      void ysURForceAppControl::vision_WaitCaptureMoveDone() {
        if (sub_step_ == 301
        ) {
          if (KDL::Equal(ys_cur_q_, ys_cameraCapture_q_, joint_eps_)
          ) {
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            ys_vision_job_done_ = false;
            sub_step_++;
          }
        }
      }
      void ysURForceAppControl::vision_SendVisionCmd() {
        if (sub_step_ == 302
        ) {
          //publish
          std_msgs::msg::Int32 msg;
          msg.data = 1;//1, vision pcl job
          vision_job_publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_++;
        }
      }
      void ysURForceAppControl::vision_WaitVisionSolverDone() {
        if (sub_step_ == 303
        ) {
          if (ys_vision_job_done_==true)
          {
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_ = 9999;
            std_msgs::msg::Int32 msg;
            msg.data = 103;//100 + vision done
            cmd_result_publisher_->publish(msg);
            app_cmd_ = AppCommand::DO_CURVE_POLISHING;
            sub_step_ = app_cmd_*100;
            RCLCPP_INFO(this->get_logger(),
                "pub the result of command: %d. ", msg.data);
            }
        }
      }
      void ysURForceAppControl::doForcePolishing() {
        switch (sub_step_)
        {
        case 400://AppCommand::DO_CURVE_POLISHING*100
          polish_goPolishBase();
          break;
        case 401:
          polish_waitPolishBase();
          break;
        case 402:
          polish_doForceContact();
          break;
        case 403:
          polish_startPolishtool();
          break;
        case 404:
          polish_doCurvePolishing();
          break;
        case 405:
          polish_endPolishtool();
          break;
        case 406:
          polish_goBackHome();
          break;
        case 407:
          polish_waitBackHome();
          break;
        default:
          break;
        }

      }
      void ysURForceAppControl::polish_goPolishBase() {
        if (400==sub_step_) {
          // trajectory
          trajectory_msgs::msg::JointTrajectory traj_goal;
          traj_goal.header.stamp = this->now();
          traj_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              traj_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          
          rclcpp::Duration moveT(2*speed_level_, 1E9/125);
          //base point
          trajectory_msgs::msg::JointTrajectoryPoint polish_base_point;
          for(int i=0;i<joint_size_;++i)
          {
            polish_base_point.positions.push_back(ys_polishBase_q_(i));
            polish_base_point.velocities.push_back(0);
          }
          polish_base_point.time_from_start = moveT;
          traj_goal.points.push_back(polish_base_point);

          //start up point
          KDL::Frame  startPos, upFrame;
          startPos = frame_polishcloud_transform_* polishcurve_OriginFrames_[0];
          // startPos = frame_polishcloud_transform_*frame_polishcloud_base_ * polishcurve_OriginFrames_[0];
          //ik
          KDL::Frame moveFrame;
          moveFrame.p = KDL::Vector(0,0,dz_polish_startup_tool_);//offset tool z  
          moveFrame.M = KDL::Rotation::RPY(0,0,0);
          upFrame = startPos * moveFrame;
          KDL::JntArray ys_resultJnt(joint_size_);
          int rc = ys_tcp_tracik_solver_->CartToJnt(ys_polishBase_q_, upFrame, ys_resultJnt);
          // IK 失败(rc!=1)时 ys_resultJnt 是未初始化垃圾；构型跳变时机械臂会猛甩。
          // 两种情况都中止流程，绝不把目标发出去（2026-07-27 悬空狂晃事故的修复）
          // 分关节检查: 关节1~5 限 90°（防翻肩/翻肘/翻腕），wrist_3 放宽到 170°——
          // 打磨头是旋转体，工具绕自身 z 多转不影响功能（2026-07-27 误伤修复）
          bool branch_jump = false;
          if (rc == 1) {
            for (int j = 0; j < joint_size_; j++) {
              double lim = (j == joint_size_ - 1) ? M_PI * 0.95 : M_PI / 2;
              if (std::fabs(ys_polishBase_q_(j) - ys_resultJnt(j)) > lim) { branch_jump = true; break; }
            }
          }
          if (rc != 1 || branch_jump) {
            RCLCPP_ERROR(this->get_logger(),
              "polish start up target unreachable or branch jump (rc=%d, branch_jump=%d), polish aborted. "
              "Check vision frame / polishBase seed.", rc, (int)branch_jump);
            RCLCPP_ERROR(this->get_logger()," seed(polishBase), %f, %f, %f, %f, %f, %f",
              ys_polishBase_q_(0)*180/M_PI, ys_polishBase_q_(1)*180/M_PI, ys_polishBase_q_(2)*180/M_PI, ys_polishBase_q_(3)*180/M_PI, ys_polishBase_q_(4)*180/M_PI, ys_polishBase_q_(5)*180/M_PI);
            RCLCPP_ERROR(this->get_logger()," ik resultJnt,    %f, %f, %f, %f, %f, %f",
              ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
            abortPolishing("打磨预备目标 IK 不可达或发生跳支", false);
            return;
          }
          RCLCPP_INFO(this->get_logger()," polish start up target, %f, %f, %f, %f, %f, %f", 
            ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
          //polish start up point
          rclcpp::Duration deltaT(2*speed_level_, 1E9/125);
          trajectory_msgs::msg::JointTrajectoryPoint ys_up_point;
          for(int i=0;i<joint_size_;++i)
          {
              ys_up_point.positions.push_back(ys_resultJnt(i));
              ys_up_point.velocities.push_back(0);
          }
          ys_up_point.time_from_start = moveT + deltaT;// 第二点时间必须晚于第一点，否则被控制器拒绝
          traj_goal.points.push_back(ys_up_point);

          //pub
          RCLCPP_INFO(this->get_logger(),"pub go polish base trajectory");
          ys_traj_publisher_->publish(traj_goal);   

          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_++;
        }
      }
      double ysURForceAppControl::maxJointSpeed() const {
        double max_speed = 0.0;
        for (int i = 0; i < joint_size_; ++i) {
          max_speed = std::max(max_speed, std::fabs(ys_cur_vel(i)));
        }
        return max_speed;
      }

      void ysURForceAppControl::resetForceContactState() {
        contact_settle_started_ = false;
        contact_tare_collecting_ = false;
        contact_tare_count_ = 0;
        contact_tare_sum_ = 0.0;
        contact_tare_sum_fx_ = 0.0;
        contact_tare_sum_fy_ = 0.0;
        contact_fz_zero_ = 0.0;
        contact_fx_zero_ = 0.0;
        contact_fy_zero_ = 0.0;
        contact_detection_enabled_ = false;
        contact_confirm_count_ = 0;
        contact_confirmed_ = false;
        contact_approach_started_ = false;
        approach_retare_hold_started_ = false;
        approach_retare_done_ = false;
        approach_lateral_started_ = false;
        approach_lateral_done_ = false;
        contact_hold_started_ = false;
        contact_ik_failure_count_ = 0;
        force_mode_verify_stable_ = false;
        force_mode_overforce_active_ = false;
        force_mode_hard_overforce_active_ = false;
        force_mode_lateral_overforce_active_ = false;
        last_polish_step_ = 0;
        polish_tangential_started_ = false;
        force_history_.clear();
        polish_tool_open_pending_ = false;
        polish_tool_open_done_ = false;
        polish_tool_open_ok_ = false;
        polish_tool_close_pending_ = false;
        polish_tool_close_done_ = false;
        polish_tool_close_ok_ = false;
        debug_approach_started_ = false;
        control_dt_index_ = 0;
        // 每次新打磨命令必须从轨迹起点开始，否则上次中断残留的步进索引
        // 会让 404 直接从轨迹中段"续跑"，导致 IK 解与当前姿态跳变而中止。
        polishcurve_step_index_ = 0;
        sidepolish_step_index_ = 0;
        polishcurve_yindex_ = 0;
        frame_forceadjust_base_.p = KDL::Vector(0, 0, 0);
        frame_forceadjust_base_.M = KDL::Rotation::Identity();
        orient_adapt_rx_ = 0.0;
        orient_adapt_ry_ = 0.0;
        feed_gate_hold_count_ = 0;
        polish_end_disable_sent_ = false;
      }

      void ysURForceAppControl::publishCurrentPositionHold(double duration_sec) {
        trajectory_msgs::msg::JointTrajectory hold;
        hold.header.stamp = this->now();
        hold.header.frame_id = "force_contact_hold";
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int i = 0; i < joint_size_; ++i) {
          hold.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          point.positions.push_back(ys_cur_q_(i));
          point.velocities.push_back(0.0);
        }
        point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.05, duration_sec));
        hold.points.push_back(point);
        ys_traj_publisher_->publish(hold);
      }

      void ysURForceAppControl::publishOrientationHold(const KDL::Rotation &rot, double duration_sec) {
        // 位置保持当前TCP点、姿态转到 rot: 402逼近前需把姿态从示教值转到视觉拟合值再重新去皮。
        KDL::JntArray targetJnt(joint_size_);
        const KDL::Frame target(rot, ys_curP_tcp_.p);
        if (ys_tcp_tracik_solver_->CartToJnt(ys_cur_q_, target, targetJnt) != 1) {
          RCLCPP_WARN(this->get_logger(), "orientation hold IK failed; holding current joints");
          publishCurrentPositionHold(duration_sec);
          return;
        }
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.header.frame_id = "orientation_hold";
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int i = 0; i < joint_size_; ++i) {
          traj.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          point.positions.push_back(targetJnt(i));
          point.velocities.push_back(0.0);
        }
        point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.05, duration_sec));
        traj.points.push_back(point);
        ys_traj_publisher_->publish(traj);
      }

      void ysURForceAppControl::polish_waitPolishBase() {
        if (sub_step_ != 401) {
          return;
        }

        KDL::Frame startPos = frame_polishcloud_transform_ * polishcurve_OriginFrames_[0];
        KDL::Frame moveFrame;
        moveFrame.p = KDL::Vector(0, 0, dz_polish_startup_tool_);
        moveFrame.M = KDL::Rotation::Identity();
        const KDL::Frame upFrame = startPos * moveFrame;
        KDL::JntArray targetJnt(joint_size_);
        const int rc = ys_tcp_tracik_solver_->CartToJnt(ys_polishBase_q_, upFrame, targetJnt);
        if (rc != 1) {
          RCLCPP_ERROR(this->get_logger(), "polish start-up IK failed while waiting; polish aborted");
          abortPolishing("等待打磨预备位时 IK 求解失败", false);
          return;
        }

        const bool pose_reached = KDL::Equal(ys_cur_q_, targetJnt, joint_eps_);
        const bool robot_still = maxJointSpeed() <= contact_joint_velocity_limit_;
        if (!pose_reached || !robot_still) {
          contact_settle_started_ = false;
          contact_tare_collecting_ = false;
          contact_tare_count_ = 0;
          contact_tare_sum_ = 0.0;
          contact_tare_sum_fx_ = 0.0;
          contact_tare_sum_fy_ = 0.0;
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!contact_settle_started_) {
          contact_settle_started_ = true;
          contact_settle_start_ = now;
          RCLCPP_INFO(this->get_logger(),
            "polish start-up reached; settling %.2fs before local force tare", contact_settle_time_);
          return;
        }
        if (std::chrono::duration<double>(now - contact_settle_start_).count() < contact_settle_time_) {
          return;
        }

        if (contact_tare_count_ == 0 && !contact_tare_collecting_) {
          contact_tare_sum_ = 0.0;
          contact_tare_sum_fx_ = 0.0;
          contact_tare_sum_fy_ = 0.0;
          contact_tare_collecting_ = true;
          RCLCPP_INFO(this->get_logger(), "collecting %d local force-tare samples", contact_tare_samples_);
          return;
        }
        if (contact_tare_collecting_) {
          return;
        }

        contact_detection_enabled_ = true;
        contact_confirm_count_ = 0;
        contact_confirmed_ = false;
        contact_approach_started_ = false;
        RCLCPP_INFO(this->get_logger(), "sub step done: %d; local force tare=%.3f N",
                    sub_step_, contact_fz_zero_);
        sub_step_++;
      }

      void ysURForceAppControl::sendEnableForceModeRequest() {
        if (!use_force_mode_ || force_mode_enabled_ || force_mode_enable_pending_) {
          return;
        }
        if (!force_mode_client_) {
          RCLCPP_ERROR(this->get_logger(), "force mode client not created");
          force_mode_enable_pending_ = true;
          force_mode_enable_done_ = true;
          force_mode_enable_ok_ = false;
          return;
        }
        if (!force_mode_client_->service_is_ready()) {
          if (!force_mode_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(),
                         "force mode service /force_mode_server/set_force_mode not available");
            force_mode_enable_pending_ = true;
            force_mode_enable_done_ = true;
            force_mode_enable_ok_ = false;
            return;
          }
        }
        auto req = std::make_shared<eli_common_interface::srv::ForceMode::Request>();
        req->enable = true;
        // 力控参考系 = 接触瞬间的 tool0 位姿(相对 base)。直线打磨工具姿态恒定，
        // FIX 模式固定参考系即可保证 z 轴始终沿打磨头轴线(压紧方向)。
        double rx = 0.0, ry = 0.0, rz = 0.0;
        ys_curP_tcp_.M.GetRPY(rx, ry, rz);
        req->reference_frame = {ys_curP_tcp_.p.x(), ys_curP_tcp_.p.y(), ys_curP_tcp_.p.z(), rx, ry, rz};
        // 2026-08-02: 可选开放 Rx/Ry 旋转柔顺(目标力矩0)。板法向与工具轴不对齐时
        // 砂轮边缘接触产生 Mx/My 与侧向拖曳；零力矩目标让端面缓慢贴合板面卸力。
        // 角速度受 limits 限制(默认0.05rad/s≈3°/s)，稳态转角≈板子倾角，无突变。
        // Rz 保持位置模式，扫掠方向不受影响。
        const int rot_sel = force_mode_rot_compliance_ ? 1 : 0;
        req->selection_vector = {0, 0, 1, rot_sel, rot_sel, 0};  // z 轴力控 + 可选 Rx/Ry 柔顺
        req->wrench = {0, 0, force_mode_wrench_z_, 0, 0, 0};
        req->mode = 0;  // FIX: 力控坐标系 = 参考坐标系
        req->limits = {0, 0, force_mode_z_vel_limit_,
                       rot_sel * force_mode_rot_vel_limit_, rot_sel * force_mode_rot_vel_limit_, 0};
        force_mode_axis_base_ = ys_curP_tcp_.M * KDL::Vector(0, 0, 1);
        const double axis_norm = force_mode_axis_base_.Norm();
        if (axis_norm > 1e-9) {
          force_mode_axis_base_ = force_mode_axis_base_ / axis_norm;
        }
        RCLCPP_INFO(this->get_logger(),
                    "force mode command sent: ref(x,y,z,rx,ry,rz)=(%.4f,%.4f,%.4f,%.3f,%.3f,%.3f) "
                    "axis_base=(%.4f,%.4f,%.4f) applied_Fz=%.2f vlim=%.4f rot_comp=%d wlim=%.4f",
                    req->reference_frame[0], req->reference_frame[1], req->reference_frame[2],
                    req->reference_frame[3], req->reference_frame[4], req->reference_frame[5],
                    force_mode_axis_base_.x(), force_mode_axis_base_.y(), force_mode_axis_base_.z(),
                    force_mode_wrench_z_, force_mode_z_vel_limit_,
                    force_mode_rot_compliance_ ? 1 : 0, force_mode_rot_vel_limit_);
        // 异步发送: 单线程 executor 不能在这里同步等 future(会死锁)，
        // 响应由 executor 在回调间处理, 置 done 标志, 402 主流程逐拍轮询。
        force_mode_client_->async_send_request(
            req,
            [this](rclcpp::Client<eli_common_interface::srv::ForceMode>::SharedFuture future) {
              // 已超时/退刀/切换命令时，迟到的 start 响应不得重新污染本地状态。
              if (!force_mode_enable_pending_) {
                RCLCPP_WARN(this->get_logger(), "ignoring stale force mode enable response");
                return;
              }
              try {
                auto res = future.get();
                force_mode_enable_ok_ = res->success;
                if (!res->success) {
                  RCLCPP_ERROR(this->get_logger(), "enable force mode failed: %s", res->message.c_str());
                }
              } catch (const std::exception& e) {
                force_mode_enable_ok_ = false;
                RCLCPP_ERROR(this->get_logger(), "enable force mode exception: %s", e.what());
              }
              force_mode_enable_done_ = true;
              RCLCPP_INFO(this->get_logger(), "force mode enable response received, ok=%d",
                          (int)force_mode_enable_ok_);
            });
        force_mode_enable_pending_ = true;
        force_mode_enable_done_ = false;
        force_mode_enable_start_ = std::chrono::steady_clock::now();
        // 新一轮力控开始, 清除上一轮 disable 的完成痕迹, 避免 405 误用陈旧 done 直接放行
        force_mode_disable_pending_ = false;
        force_mode_disable_done_ = false;
      }

      bool ysURForceAppControl::disableForceMode() {
        const bool command_may_be_active = force_mode_enabled_ || force_mode_enable_pending_ || force_mode_enable_ok_;
        if (!command_may_be_active) {
          // 没有活动力控: 无可等待的切换, 直接置完成(405 的静置计时从当前起算)
          force_mode_disable_pending_ = false;
          force_mode_disable_done_ = true;
          force_mode_disable_ok_ = true;
          force_mode_disable_done_time_ = std::chrono::steady_clock::now();
          return true;
        }
        force_mode_enabled_ = false;
        force_mode_enable_pending_ = false;
        force_mode_enable_done_ = false;
        force_mode_enable_ok_ = false;
        if (!force_mode_client_ || !force_mode_client_->service_is_ready()) {
          // 服务不可用(驱动重启等)时直接清状态，不阻塞退刀
          force_mode_disable_pending_ = false;
          force_mode_disable_done_ = true;
          force_mode_disable_ok_ = false;
          force_mode_disable_done_time_ = std::chrono::steady_clock::now();
          return false;
        }
        auto req = std::make_shared<eli_common_interface::srv::ForceMode::Request>();
        req->enable = false;
        // 不阻塞等待(单线程 executor 会死锁), 但跟踪响应: 405 退刀需等响应到达
        // 并静置, 让驱动侧控制器完成 deactivate/activate 切换后再发轨迹。
        force_mode_disable_pending_ = true;
        force_mode_disable_done_ = false;
        force_mode_disable_ok_ = false;
        force_mode_client_->async_send_request(req,
          [this](rclcpp::Client<eli_common_interface::srv::ForceMode>::SharedFuture future) {
            if (!force_mode_disable_pending_) {
              return;
            }
            try {
              auto res = future.get();
              force_mode_disable_ok_ = res->success;
              if (!res->success) {
                RCLCPP_ERROR(this->get_logger(),
                  "disable force mode failed: %s", res->message.c_str());
              }
            } catch (const std::exception &e) {
              force_mode_disable_ok_ = false;
              RCLCPP_WARN(this->get_logger(), "force mode disable response error: %s", e.what());
            }
            force_mode_disable_pending_ = false;
            force_mode_disable_done_ = true;
            force_mode_disable_done_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "force mode disable response received");
          });
        RCLCPP_INFO(this->get_logger(), "force mode disable request sent");
        return true;
      }

      void ysURForceAppControl::polish_doForceContact() {
        if (sub_step_ == 402 ) {
          // 调试空跑模式: 不做接触下压，但先发一条平滑轨迹慢速走到轨迹起点——
          // 直接跳进 404 会被 20ms 流式目标全速追点（2026-07-27 用户反馈接近速度过快）
          if (debug_skip_force_contact_) {
            if (!debug_approach_started_) {
              KDL::Frame startPos = frame_polishcloud_transform_ * polishcurve_OriginFrames_[0];
              KDL::JntArray targetJnt(joint_size_);
              int rc = ys_tcp_tracik_solver_->CartToJnt(ys_cur_q_, startPos, targetJnt);
              if (rc != 1) {
                RCLCPP_ERROR(this->get_logger(),"DEBUG approach IK failed (rc=%d), abort", rc);
                abortPolishing("空跑接近目标 IK 求解失败", false);
                return;
              }
              trajectory_msgs::msg::JointTrajectory traj;
              traj.header.stamp = this->now();
              for(int i=0;i<joint_size_;++i) {
                traj.joint_names.push_back(ys_prefix_ + joint_names_[i]);
              }
              trajectory_msgs::msg::JointTrajectoryPoint pt;
              for(int i=0;i<joint_size_;++i) {
                pt.positions.push_back(targetJnt(i));
                pt.velocities.push_back(0);
              }
              pt.time_from_start = rclcpp::Duration::from_seconds(debug_approach_time_);
              traj.points.push_back(pt);
              ys_traj_publisher_->publish(traj);
              debug_approach_target_q_ = targetJnt;
              debug_approach_started_ = true;
              RCLCPP_INFO(this->get_logger(),"DEBUG: smooth approach to polish start (%.1fs)", debug_approach_time_);
            } else if (KDL::Equal(ys_cur_q_, debug_approach_target_q_, joint_eps_)) {
              debug_approach_started_ = false;
              RCLCPP_INFO(this->get_logger(),"DEBUG: approach done, goto polishing");
              sub_step_++;
            }
            return;
          }

          // 力控命令已发出: 等待 SDK socket 写入结果(不是控制器闭环状态确认)。
          if (force_mode_enable_pending_) {
            if (!force_mode_enable_done_) {
              // 看门狗: 等待超过 3s 视为失败
              auto elapsed = std::chrono::steady_clock::now() - force_mode_enable_start_;
              if (elapsed > std::chrono::seconds(3)) {
                RCLCPP_ERROR(this->get_logger(), "enable force mode: response timeout, polish aborted");
                abortPolishing("启用力控服务响应超时", true);
              }
              return;
            }
            force_mode_enable_pending_ = false;
            if (!force_mode_enable_ok_) {
              RCLCPP_ERROR(this->get_logger(), "enable force mode failed, polish aborted");
              abortPolishing("启用力控失败", true);
              return;
            }
            force_mode_enabled_ = true;
            force_mode_last_contact_ = std::chrono::steady_clock::now();
            force_mode_verify_start_ = force_mode_last_contact_;
            force_mode_verify_last_log_ = force_mode_last_contact_;
            force_mode_verify_stable_ = false;
            RCLCPP_INFO(this->get_logger(),
              "force mode command accepted by SDK; verifying measured force before polishing");
            return;
          }

          // SDK 返回值只代表命令已写入 socket。先保持当前位置，确认控制器确实建立了
          // 目标法向力，再启动打磨机和切向轨迹；这样符号/参考系/目标过小等问题不会
          // 等到机械臂已经在板面上移动后才暴露。
          if (force_mode_enabled_) {
            const auto now = std::chrono::steady_clock::now();
            const double relative_fz = relFz();
            const double expected_reaction_fz = force_mode_sensor_target_fz_;
            const double force_error = std::fabs(relative_fz - expected_reaction_fz);

            if (std::chrono::duration<double>(now - force_mode_verify_last_log_).count() >= 0.5) {
              RCLCPP_INFO(this->get_logger(),
                "force-mode verification progress: relative_fz=%.3f sensor_target=%.3f "
                "sdk_applied_Fz=%.3f error=%.3f",
                relative_fz, expected_reaction_fz, force_mode_wrench_z_, force_error);
              force_mode_verify_last_log_ = now;
            }

            if (relative_fz <= force_mode_abort_fz_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode verification: excessive force %.3f N; aborting before polish", relative_fz);
              abortPolishing("力控建立前检测到过力", true);
              return;
            }

            if (force_error <= force_mode_verify_tolerance_) {
              if (!force_mode_verify_stable_) {
                force_mode_verify_stable_ = true;
                force_mode_verify_stable_start_ = now;
                RCLCPP_INFO(this->get_logger(),
                  "force-mode verification entered target band: relative_fz=%.3f target=%.3f tol=%.3f",
                  relative_fz, expected_reaction_fz, force_mode_verify_tolerance_);
              }
              const double stable_for = std::chrono::duration<double>(
                now - force_mode_verify_stable_start_).count();
              if (stable_for >= force_mode_verify_time_) {
                force_mode_last_contact_ = now;
                force_mode_monitor_last_log_ = now;
                force_mode_verify_stable_ = false;
                force_mode_overforce_active_ = false;
                force_mode_hard_overforce_active_ = false;
                force_mode_lateral_overforce_active_ = false;
                RCLCPP_INFO(this->get_logger(),
                  "force mode VERIFIED: relative_fz=%.3f stable_for=%.2fs; sub step done: %d",
                  relative_fz, stable_for, sub_step_);
                sub_step_++;
              }
            } else {
              force_mode_verify_stable_ = false;
            }

            const double verify_elapsed = std::chrono::duration<double>(
              now - force_mode_verify_start_).count();
            if (verify_elapsed > force_mode_verify_timeout_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode verification timeout: relative_fz=%.3f expected=%.3f +/- %.3f; "
                "aborting before tangential motion",
                relative_fz, expected_reaction_fz, force_mode_verify_tolerance_);
              abortPolishing("力控在规定时间内未稳定", true);
            }
            return;
          }

          const KDL::Frame startPos = frame_polishcloud_transform_ * polishcurve_OriginFrames_[0];
          KDL::Frame curFrame = ys_curP_tcp_;
          curFrame.M = startPos.M;
          const double relative_fz = relFz();

          // 2026-08-02: 逼近姿态(startPos.M, 视觉拟合)与预打磨示教姿态可能相差很大,
          // 401在示教姿态下的去皮重力残差会随手腕转动转进z轴形成虚力(实机+10N量级,
          // 掩盖真实接触→检测不到→压到碰撞报警)。逼近开始前先在视觉姿态下重新去皮。
          if (!approach_retare_done_) {
            const auto now = std::chrono::steady_clock::now();
            if (!approach_retare_hold_started_) {
              approach_retare_hold_started_ = true;
              approach_retare_hold_start_ = now;
              // 必须清零计数: 否则401的旧计数会让下面的完成判断直接通过、跳过重新去皮
              contact_tare_count_ = 0;
              contact_tare_sum_ = 0.0;
              contact_tare_sum_fx_ = 0.0;
              contact_tare_sum_fy_ = 0.0;
              publishOrientationHold(startPos.M, contact_settle_time_ + 0.5);
              RCLCPP_INFO(this->get_logger(),
                "402 pre-approach: rotating to vision orientation for re-tare");
              return;
            }
            const double hold_elapsed = std::chrono::duration<double>(
              now - approach_retare_hold_start_).count();
            if (hold_elapsed > 15.0) {
              RCLCPP_WARN(this->get_logger(),
                "402 pre-approach re-tare: settle timeout; continuing with 401 tare");
              approach_retare_done_ = true;
              return;
            }
            if (contact_tare_collecting_) {
              return;  // 采集中, 等回调凑满25帧并自动清collecting
            }
            if (contact_tare_count_ >= contact_tare_samples_) {
              approach_retare_done_ = true;
              RCLCPP_INFO(this->get_logger(),
                "402 pre-approach re-tare done: zero=(%.3f,%.3f,%.3f)",
                contact_fx_zero_, contact_fy_zero_, contact_fz_zero_);
              return;
            }
            if (hold_elapsed >= contact_settle_time_
                && maxJointSpeed() <= contact_joint_velocity_limit_) {
              contact_tare_collecting_ = true;
              RCLCPP_INFO(this->get_logger(),
                "402 pre-approach: collecting %d re-tare samples at vision orientation",
                contact_tare_samples_);
            }
            return;
          }

          if (!contact_approach_started_) {
            contact_approach_started_ = true;
            contact_approach_start_ = std::chrono::steady_clock::now();
            contact_approach_start_p_ = ys_curP_tcp_.p;
            contact_approach_axis_base_ = startPos.M * KDL::Vector(0, 0, 1);
            const double norm = contact_approach_axis_base_.Norm();
            if (norm > 1e-9) {
              contact_approach_axis_base_ = contact_approach_axis_base_ / norm;
            }
            RCLCPP_INFO(this->get_logger(),
              "402 approach started: axis_base=(%.4f,%.4f,%.4f), max_travel=%.3fm timeout=%.1fs",
              contact_approach_axis_base_.x(), contact_approach_axis_base_.y(),
              contact_approach_axis_base_.z(), contact_max_travel_, contact_timeout_);
          }

          // 接触确认后立即用当前位置目标抢占逼近轨迹；不再反向发布 -3mm 轨迹。
          // 等待保持轨迹和实际关节速度稳定后，才把同一轴交给 startForceMode。
          if (contact_confirmed_) {
            if (!contact_hold_started_) {
              contact_hold_started_ = true;
              contact_hold_start_ = std::chrono::steady_clock::now();
              publishCurrentPositionHold(contact_hold_time_);

              KDL::Frame offsetFrame;
              offsetFrame.p = ys_curP_tcp_.p - startPos.p;
              offsetFrame.M = KDL::Rotation::Identity();
              frame_forceadjust_base_ = offsetFrame * frame_forceadjust_base_;
              RCLCPP_INFO(this->get_logger(),
                "contact confirmed (%d frames): relative_fz=%.3f raw_fz=%.3f; holding before force mode",
                contact_confirm_count_, relative_fz, ys_contact_wrench_sensor_.force.data[2]);
              RCLCPP_INFO(this->get_logger(), "contact offset base=(%.5f,%.5f,%.5f)",
                offsetFrame.p.x(), offsetFrame.p.y(), offsetFrame.p.z());
              return;
            }

            const double hold_elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - contact_hold_start_).count();
            // time_from_start 到时不等于 action 已经完成。实测 result 回调约晚 50ms；
            // 在完成余量结束前调用 startForceMode，会被随后发生的控制流程切换覆盖。
            if (hold_elapsed < contact_hold_time_ + contact_hold_completion_margin_
                || maxJointSpeed() > contact_joint_velocity_limit_) {
              return;
            }

            if (use_force_mode_) {
              sendEnableForceModeRequest();
              if (!force_mode_enable_pending_) {
                RCLCPP_ERROR(this->get_logger(), "failed to send force mode command; polish aborted");
                abortPolishing("力控命令发送失败", true);
              }
              return;
            }

            RCLCPP_INFO(this->get_logger(), "sub step done: %d (legacy position force adjustment)", sub_step_);
            sub_step_++;
            return;
          }

          const KDL::Vector approach_delta = ys_curP_tcp_.p - contact_approach_start_p_;
          const double approach_travel =
            approach_delta.x() * contact_approach_axis_base_.x() +
            approach_delta.y() * contact_approach_axis_base_.y() +
            approach_delta.z() * contact_approach_axis_base_.z();
          const double approach_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - contact_approach_start_).count();
          if (approach_travel > contact_max_travel_ || approach_elapsed > contact_timeout_) {
            publishCurrentPositionHold(contact_hold_time_);
            contact_detection_enabled_ = false;
            RCLCPP_ERROR(this->get_logger(),
              "contact search watchdog: travel=%.4fm (limit %.4f), time=%.2fs (limit %.2f); polish aborted",
              approach_travel, contact_max_travel_, approach_elapsed, contact_timeout_);
            abortPolishing("接触搜索超过最大行程或超时", true);
            return;
          }

          control_dt_index_++;
          if (control_dt_index_ != control_dt_count_) {
            return;
          }
          control_dt_index_=0;

          RCLCPP_INFO(this->get_logger(),
            "402 approach: step=%.5f travel=%.4f relative_fz=%.3f raw_fz=%.3f confirm=%d/%d",
            contact_step_, approach_travel, relative_fz, ys_contact_wrench_sensor_.force.data[2],
            contact_confirm_count_, contact_confirm_samples_);

          // 2026-08-02 侧向对中(独立平移段): 一次性开环平移到名义起点正上方(保持轴向高度),
          // 到位后再进入纯轴向下压。取代此前的每拍反馈斜插——跟踪延迟使步进方向在目标线
          // 两侧反复翻转, 实机表现为板上方原地扭摆不下降(用户两轮实机复现)。
          if (!approach_lateral_done_) {
            const KDL::Vector to_start = startPos.p - ys_curP_tcp_.p;
            const double axial_rem = to_start.x() * contact_approach_axis_base_.x()
              + to_start.y() * contact_approach_axis_base_.y()
              + to_start.z() * contact_approach_axis_base_.z();
            const KDL::Vector lateral(
              to_start.x() - axial_rem * contact_approach_axis_base_.x(),
              to_start.y() - axial_rem * contact_approach_axis_base_.y(),
              to_start.z() - axial_rem * contact_approach_axis_base_.z());
            const double lateral_norm = lateral.Norm();
            if (lateral_norm <= 0.005) {
              approach_lateral_done_ = true;
              return;
            }
            const auto now = std::chrono::steady_clock::now();
            if (!approach_lateral_started_) {
              approach_lateral_started_ = true;
              approach_lateral_start_ = now;
              KDL::JntArray latJnt(joint_size_);
              const KDL::Frame latTarget(curFrame.M, ys_curP_tcp_.p + lateral);
              if (ys_tcp_tracik_solver_->CartToJnt(ys_cur_q_, latTarget, latJnt) != 1) {
                RCLCPP_WARN(this->get_logger(),
                  "402 lateral align IK failed; descending without lateral correction");
                approach_lateral_done_ = true;
                return;
              }
              trajectory_msgs::msg::JointTrajectory lat_goal;
              lat_goal.header.stamp = this->now();
              lat_goal.header.frame_id = "new";
              trajectory_msgs::msg::JointTrajectoryPoint lat_pt;
              for (int i = 0; i < joint_size_; ++i) {
                lat_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
                lat_pt.positions.push_back(latJnt(i));
                lat_pt.velocities.push_back(0.0);
              }
              // 10mm/s 平移, 限时1~6s
              const double move_t = std::min(6.0, std::max(1.0, lateral_norm / 0.01));
              lat_pt.time_from_start = rclcpp::Duration::from_seconds(move_t);
              lat_goal.points.push_back(lat_pt);
              ys_traj_publisher_->publish(lat_goal);
              RCLCPP_INFO(this->get_logger(),
                "402 lateral align: offset=%.1fmm, moving above nominal start (%.1fs)",
                lateral_norm * 1000.0, move_t);
              return;
            }
            if (std::chrono::duration<double>(now - approach_lateral_start_).count() > 15.0) {
              RCLCPP_WARN(this->get_logger(),
                "402 lateral align timeout (offset=%.1fmm); descending anyway",
                lateral_norm * 1000.0);
              approach_lateral_done_ = true;
            }
            return;
          }

          // 这里只需求本周期的笛卡尔端点。原实现每周期求 60 次 IK，不仅会阻塞
          // 约数百毫秒，任意一个 5ms TRAC-IK 超时还会让整个流程退出。
          // 轨迹控制器会在当前位置和这个 1mm 端点之间自行平滑插值。
          KDL::JntArray targetJnt(joint_size_);
          int last_rc = -1;
          int solved_attempt = -1;
          double solved_step = contact_step_;
          for (int attempt = 0; attempt < 3; ++attempt) {
            const double step_scale = 1.0 / static_cast<double>(1 << attempt);
            KDL::Frame moveFrame;
            moveFrame.p = KDL::Vector(0, 0, contact_step_ * step_scale);
            moveFrame.M = KDL::Rotation::Identity();
            const KDL::Frame targetFrame = curFrame * moveFrame;
            targetJnt = ys_cur_q_;
            last_rc = ys_tcp_tracik_solver_->CartToJnt(ys_cur_q_, targetFrame, targetJnt);
            if (last_rc == 1 && KDL::Equal(ys_cur_q_, targetJnt, M_PI / 10)) {
              solved_attempt = attempt;
              solved_step = contact_step_ * step_scale;
              break;
            }
          }

          if (solved_attempt < 0) {
            contact_ik_failure_count_++;
            RCLCPP_WARN(this->get_logger(),
              "contact endpoint IK transient failure: rc=%d consecutive=%d/%d; will retry",
              last_rc, contact_ik_failure_count_, contact_ik_max_failures_);
            if (contact_ik_failure_count_ >= contact_ik_max_failures_) {
              publishCurrentPositionHold(contact_hold_time_);
              contact_detection_enabled_ = false;
              RCLCPP_ERROR(this->get_logger(),
                "contact endpoint IK failed for %d consecutive cycles; polish aborted",
                contact_ik_failure_count_);
              abortPolishing("接触搜索连续 IK 求解失败", true);
            }
            return;
          }
          contact_ik_failure_count_ = 0;
          if (solved_attempt > 0) {
            RCLCPP_WARN(this->get_logger(),
              "contact endpoint IK used reduced step %.6fm after retry", solved_step);
          }

          trajectory_msgs::msg::JointTrajectory ys_goal;
          ys_goal.header.stamp = this->now();
          ys_goal.header.frame_id = "contact_approach";
          trajectory_msgs::msg::JointTrajectoryPoint point;
          for (int i = 0; i < joint_size_; ++i) {
            ys_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
            point.positions.push_back(targetJnt(i));
            point.velocities.push_back(0.0);
          }
          point.time_from_start = rclcpp::Duration::from_seconds(0.48);
          ys_goal.points.push_back(point);
          ys_traj_publisher_->publish(ys_goal);
        }
      }
      void ysURForceAppControl::polish_startPolishtool() {
        if (sub_step_ == 403
        ) {
          const KDL::Frame nominal_start = frame_forceadjust_base_
            * frame_polishcloud_transform_ * polishcurve_OriginFrames_[0];

          // 打磨头启动期间不发切向轨迹，但仍持续执行力、失联和轴向监控。
          if (!forceModeWatchdog(nominal_start)) {
            return;
          }

          if (!polish_tool_open_pending_ && !polish_tool_open_done_) {
            ysPolishTool_Open();
            return;
          }

          if (polish_tool_open_pending_ && !polish_tool_open_done_) {
            const double wait_elapsed = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - polish_tool_open_request_start_).count();
            if (wait_elapsed > polish_tool_io_timeout_) {
              RCLCPP_ERROR(this->get_logger(),
                "Polish tool open response timeout after %.2fs; polishing aborted", wait_elapsed);
              polish_tool_open_pending_ = false;
              abortPolishing("打开打磨头 IO 响应超时", true);
            }
            return;
          }

          if (!polish_tool_open_ok_) {
            RCLCPP_ERROR(this->get_logger(), "Polish tool failed to open; polishing aborted");
            abortPolishing("打开打磨头 IO 失败", true);
            return;
          }

          const double spinup_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - polish_tool_spinup_start_).count();
          if (spinup_elapsed < polish_tool_spinup_time_) {
            return;
          }

          RCLCPP_INFO(this->get_logger(),
            "Polish tool spin-up complete: stable_for=%.2fs; starting tangential trajectory",
            spinup_elapsed);
          polish_tool_open_pending_ = false;
          polish_tool_open_done_ = false;
          polish_tool_open_ok_ = false;
          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_++;
        }
      }
      void ysURForceAppControl::polish_endPolishtool() {
        if (sub_step_ == 405
        ) {
          const auto now = std::chrono::steady_clock::now();
          if (!polish_end_disable_sent_) {
            ysPolishTool_Close();
            // 退刀前关闭力控，406 抬刀回到纯位置控制
            if (!disableForceMode()) {
              markPolishFailure("退出力控服务不可用");
            }
            polish_end_disable_sent_ = true;
            polish_end_disable_start_ = now;
            RCLCPP_INFO(this->get_logger(),
              "sub step %d: waiting force-mode disable before retract", sub_step_);
            return;
          }
          // 2026-08-02: endForceMode 触发驱动侧控制器 deactivate/activate 切换,
          // 切换途中发到 scaled 控制器的轨迹 goal 会被取消(历次"收工不退刀"根因,
          // 成功/失败仅差十几 ms 的时序竞争)。等 disable 响应到达后再静置
          // retract_disable_settle_time_, 5s 超时兜底不阻塞产线。
          const double settle = force_mode_disable_done_
            ? std::chrono::duration<double>(now - force_mode_disable_done_time_).count()
            : 0.0;
          const double total = std::chrono::duration<double>(now - polish_end_disable_start_).count();
          const double close_total = std::chrono::duration<double>(
            now - polish_tool_close_request_start_).count();
          const bool close_ready = polish_tool_close_done_
            || close_total >= polish_tool_io_timeout_;
          const bool force_ready = (force_mode_disable_done_
              && settle >= retract_disable_settle_time_) || total >= 5.0;
          if (close_ready && force_ready) {
            if (!polish_tool_close_done_) {
              polish_tool_close_pending_ = false;
              markPolishFailure("关闭打磨头 IO 响应超时");
            } else if (!polish_tool_close_ok_) {
              markPolishFailure("关闭打磨头 IO 失败");
            }
            if (!force_mode_disable_done_) {
              force_mode_disable_pending_ = false;
              markPolishFailure("退出力控响应超时");
              RCLCPP_WARN(this->get_logger(),
                "force-mode disable response timeout after %.1fs; retracting anyway", total);
            } else if (!force_mode_disable_ok_) {
              markPolishFailure("退出力控失败");
            }
            polish_end_disable_sent_ = false;
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_++;
          }
        }
      }
      void ysURForceAppControl::polish_goBackHome() {
        if (406==sub_step_) {
          // trajectory
          trajectory_msgs::msg::JointTrajectory ys_goal;
          ys_goal.header.stamp = this->now();
          ys_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              ys_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          
          //fk
          KDL::Frame  curFrame, upFrame;
          curFrame = ys_curP_tcp_;

          KDL::JntArray ys_resultJnt(joint_size_);
          ys_resultJnt = ys_cur_q_;
          KDL::JntArray lastQ(joint_size_);
          KDL::Frame moveFrame;
          int trajCount=120*speed_level_;
          rclcpp::Duration deltaT(0, 1E9/50);
          // 安全退刀分两段：先沿接触轴反向（世界 X-）离开工件，再竖直抬升。
          // 原实现写死 base Y+ 侧移 100mm，异常退出时会表现为突兀的大幅横移。
          const double axial_phase = 0.4;
          for (size_t i = 0; i < trajCount; i++)
          {
            //ys_ik
            lastQ = ys_resultJnt;
            const double progress = static_cast<double>(i + 1) / static_cast<double>(trajCount);
            if (progress <= axial_phase) {
              moveFrame.p = -force_mode_axis_base_ *
                (retract_axial_distance_ * progress / axial_phase);
            } else {
              const double lift_progress = (progress - axial_phase) / (1.0 - axial_phase);
              moveFrame.p = -force_mode_axis_base_ * retract_axial_distance_
                + world_up_in_base_ * (retract_lift_height_ * lift_progress);
            }
            moveFrame.M = KDL::Rotation::RPY(0,0,0);
            upFrame = moveFrame * curFrame;
            int rc = ys_tcp_tracik_solver_->CartToJnt(lastQ, upFrame, ys_resultJnt);
            if (rc == 1) {
              if (!KDL::Equal(lastQ, ys_resultJnt, M_PI/10)) {
                RCLCPP_FATAL(this->get_logger()," ys_resultJnt  lastQ, %f, %f, %f, %f, %f, %f", 
                  lastQ(0)*180/M_PI, lastQ(1)*180/M_PI, lastQ(2)*180/M_PI, lastQ(3)*180/M_PI, lastQ(4)*180/M_PI, lastQ(5)*180/M_PI);
                RCLCPP_FATAL(this->get_logger()," ys_resultJnt  target, %f, %f, %f, %f, %f, %f", 
                  ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
                ys_resultJnt = lastQ;
                markPolishFailure("安全退刀轨迹 IK 跳支");
              }
            } else {
              RCLCPP_INFO(this->get_logger()," back polish end up target %d, trac ik failed", i+1);
              markPolishFailure("安全退刀轨迹 IK 求解失败");
            }
            //polish end up point
            trajectory_msgs::msg::JointTrajectoryPoint tmpPt;
            for(int x=0;x<joint_size_;++x)
            {
                tmpPt.positions.push_back(ys_resultJnt(x));
                tmpPt.velocities.push_back(0);
            }
            tmpPt.time_from_start = rclcpp::Duration::from_nanoseconds(deltaT.nanoseconds() * (i + 1));// 时间必须逐点递增，否则被控制器拒绝
            ys_goal.points.push_back(tmpPt);
          }

          //home point
          trajectory_msgs::msg::JointTrajectoryPoint ys_home_point;
          for(int i=0;i<joint_size_;++i)
          {
              ys_home_point.positions.push_back(ys_home_q_(i));
              ys_home_point.velocities.push_back(0);
          }
          rclcpp::Duration homeT(2*speed_level_, 0);
          // home 点时间必须在抬刀轨迹段之后（循环段总时长 deltaT*trajCount 约 2.4s*speed_level_ 已大于 homeT）
          ys_home_point.time_from_start = rclcpp::Duration::from_nanoseconds(deltaT.nanoseconds() * trajCount) + homeT;
          ys_goal.points.push_back(ys_home_point);

          //pub
          RCLCPP_INFO(this->get_logger(),"pub back home trajectory");
          ys_traj_publisher_->publish(ys_goal);   

          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_++;
        }
      }
      
      void ysURForceAppControl::polish_waitBackHome() {
        if (KDL::Equal(ys_cur_q_, ys_home_q_, joint_eps_) 
          &&sub_step_ == 407
        ) {
          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_ = 9999;
          const int32_t result_code = polish_cancelled_ ? 205
            : (polish_failed_ ? 204 : 104);
          const std::string detail = !polish_result_detail_.empty()
            ? polish_result_detail_
            : (result_code == 104 ? "打磨成功并已回 Home2"
                                  : "打磨安全收尾并已回 Home2");
          publishPolishResult(result_code, detail);
          app_cmd_ = AppCommand::NOTHING;
          // app_cmd_ = AppCommand::DO_AGV_GO_HOME;  // Elite: no AGV
          sub_step_ = app_cmd_*100;
        }
      }

      void ysURForceAppControl::logForceDiagnostics(const std::string &reason) {
        std::string seq;
        seq.reserve(force_history_.size() * 8);
        for (double f : force_history_) {
          char buf[32];
          snprintf(buf, sizeof(buf), "%.2f,", f);
          seq += buf;
        }
        if (!seq.empty()) {
          seq.pop_back();  // 去掉末尾逗号
        }
        double elapsed = 0.0;
        if (polish_tangential_started_) {
          elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - polish_tangential_start_).count();
        }
        RCLCPP_ERROR(this->get_logger(),
          "force-mode diag: reason=%s step=%d elapsed=%.2fs recent_fz=[%s]",
          reason.c_str(), last_polish_step_, elapsed, seq.c_str());
        // 六维力诊断：停摆时输出工具系瞬时/20帧平均的完整力与力矩，以及 base 系合力。
        // 部分停摆的冲击可能来自 Fx/Fy 或力矩（侧向力/振动耦合），不能只看 Fz。
        RCLCPP_ERROR(this->get_logger(),
          "force-mode diag six-axis: inst_f=(%.2f,%.2f,%.2f) inst_t=(%.2f,%.2f,%.2f) "
          "avg_f=(%.2f,%.2f,%.2f) avg_t=(%.2f,%.2f,%.2f) base_f=(%.2f,%.2f,%.2f)",
          ys_contact_wrench_sensor_.force.data[0], ys_contact_wrench_sensor_.force.data[1],
          ys_contact_wrench_sensor_.force.data[2],
          ys_contact_wrench_sensor_.torque.data[0], ys_contact_wrench_sensor_.torque.data[1],
          ys_contact_wrench_sensor_.torque.data[2],
          ys_average_wrench_.force.data[0], ys_average_wrench_.force.data[1],
          ys_average_wrench_.force.data[2],
          ys_average_wrench_.torque.data[0], ys_average_wrench_.torque.data[1],
          ys_average_wrench_.torque.data[2],
          ys_contact_wrench_base_.force.data[0], ys_contact_wrench_base_.force.data[1],
          ys_contact_wrench_base_.force.data[2]);
      }

      bool ysURForceAppControl::forceModeWatchdog(const KDL::Frame &nominal_frame) {
        if (!force_mode_enabled_) {
          return true;
        }

        const auto now = std::chrono::steady_clock::now();
        const double relative_fz = relFz();
        const double relative_fz_avg = relFzAvg();

        // 硬保护：真实冲击会持续若干帧，单帧噪声尖峰（125Hz 力数据约 8ms）不触发。
        // 用真实时间持续确认而不是"连续 N 次检查"，因为 403/404 阶段 watchdog
        // 调用频率不同（4ms vs 40ms），按检查次数计数会对同一尖峰产生不同结果。
        if (relative_fz <= force_mode_hard_abort_fz_) {
          if (!force_mode_hard_overforce_active_) {
            force_mode_hard_overforce_active_ = true;
            force_mode_hard_overforce_start_ = now;
            RCLCPP_WARN(this->get_logger(),
              "force-mode watchdog: hard overforce candidate inst=%.3f N "
              "(limit %.3f N, avg=%.3f N); confirming %.2fs",
              relative_fz, force_mode_hard_abort_fz_, relative_fz_avg,
              force_mode_hard_abort_confirm_time_);
          } else {
            const double hard_for = std::chrono::duration<double>(
              now - force_mode_hard_overforce_start_).count();
            if (hard_for >= force_mode_hard_abort_confirm_time_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode watchdog: sustained hard overforce %.3f N for %.2fs "
                "(limit %.3f N, avg=%.3f N)",
                relative_fz, hard_for, force_mode_hard_abort_fz_, relative_fz_avg);
              logForceDiagnostics("hard_overforce");
              abortPolishing("持续瞬时硬过力", true);
              return false;
            }
          }
        } else {
          force_mode_hard_overforce_active_ = false;
        }

        // 普通 -5N 阈值使用20帧滑动平均，并要求持续一小段时间，
        // 避免打磨头电机启动振动的单次尖峰直接中止流程。
        if (relative_fz_avg <= force_mode_abort_fz_) {
          if (!force_mode_overforce_active_) {
            force_mode_overforce_active_ = true;
            force_mode_overforce_start_ = now;
            RCLCPP_WARN(this->get_logger(),
              "force-mode watchdog: averaged overforce candidate avg=%.3f N inst=%.3f N; confirming %.2fs",
              relative_fz_avg, relative_fz, force_mode_abort_confirm_time_);
          } else {
            const double overforce_for = std::chrono::duration<double>(
              now - force_mode_overforce_start_).count();
            if (overforce_for >= force_mode_abort_confirm_time_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode watchdog: sustained averaged overforce %.3f N for %.2fs "
                "(limit %.3f N, inst=%.3f N)",
                relative_fz_avg, overforce_for, force_mode_abort_fz_, relative_fz);
              logForceDiagnostics("averaged_overforce");
              abortPolishing("持续平均过力", true);
              return false;
            }
          }
        } else {
          force_mode_overforce_active_ = false;
        }

        // 2026-08-02 侧向力保护: 侧向拖曳(法向不对齐/刮边)的 Fx/Fy 通常先于 Fz 失控，
        // 等 Fz 耦合超 -5N 时打磨头已别得很紧。20帧平均侧向合力持续超阈值即按同一流程退出。
        // 必须用预备姿态去皮后的相对值: 重力模型残差实测达 5~10N，绝对值在接触状态下即误触发。
        const double lateral_f_avg = std::hypot(
          ys_average_wrench_.force.data[0] - contact_fx_zero_,
          ys_average_wrench_.force.data[1] - contact_fy_zero_);
        const double lateral_f_inst = std::hypot(
          ys_contact_wrench_sensor_.force.data[0] - contact_fx_zero_,
          ys_contact_wrench_sensor_.force.data[1] - contact_fy_zero_);
        if (lateral_f_avg >= force_mode_abort_lateral_f_) {
          if (!force_mode_lateral_overforce_active_) {
            force_mode_lateral_overforce_active_ = true;
            force_mode_lateral_overforce_start_ = now;
            RCLCPP_WARN(this->get_logger(),
              "force-mode watchdog: lateral overforce candidate avg=%.3f N inst=%.3f N; confirming %.2fs",
              lateral_f_avg, lateral_f_inst, force_mode_lateral_abort_confirm_time_);
          } else {
            const double overforce_for = std::chrono::duration<double>(
              now - force_mode_lateral_overforce_start_).count();
            if (overforce_for >= force_mode_lateral_abort_confirm_time_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode watchdog: sustained lateral overforce %.3f N for %.2fs "
                "(limit %.3f N, inst=%.3f N)",
                lateral_f_avg, overforce_for, force_mode_abort_lateral_f_, lateral_f_inst);
              logForceDiagnostics("lateral_overforce");
              abortPolishing("持续侧向过力", true);
              return false;
            }
          }
        } else {
          force_mode_lateral_overforce_active_ = false;
        }

        if (relative_fz <= force_mode_min_contact_fz_) {
          force_mode_last_contact_ = now;
        } else {
          const double lost_for = std::chrono::duration<double>(now - force_mode_last_contact_).count();
          if (lost_for > force_mode_contact_loss_timeout_) {
            RCLCPP_ERROR(this->get_logger(),
              "force-mode watchdog: contact lost for %.2fs (relative_fz=%.3f N)",
              lost_for, relative_fz);
            logForceDiagnostics("contact_lost");
            abortPolishing("打磨过程中持续失去接触", true);
            return false;
          }
        }

        const KDL::Vector pose_error = ys_curP_tcp_.p - nominal_frame.p;
        const double axial_error =
          pose_error.x() * force_mode_axis_base_.x() +
          pose_error.y() * force_mode_axis_base_.y() +
          pose_error.z() * force_mode_axis_base_.z();

        if (std::chrono::duration<double>(now - force_mode_monitor_last_log_).count()
            >= force_mode_monitor_log_period_) {
          const double used_ratio = force_mode_max_axial_deviation_ > 1e-9
            ? std::fabs(axial_error) / force_mode_max_axial_deviation_ : 0.0;
          const double lat_x = ys_average_wrench_.force.data[0] - contact_fx_zero_;
          const double lat_y = ys_average_wrench_.force.data[1] - contact_fy_zero_;
          RCLCPP_INFO(this->get_logger(),
            "force-mode monitor: relative_fz=%.3f lat=(%.2f,%.2f)|%.2f| tq=(%.3f,%.3f) "
            "adapt=(%.2f,%.2f)deg axial_comp=%.4fm limit=%.4fm (%.0f%%)",
            relative_fz, lat_x, lat_y, lateral_f_avg,
            ys_average_wrench_.torque.data[0], ys_average_wrench_.torque.data[1],
            orient_adapt_rx_ * 180.0 / M_PI, orient_adapt_ry_ * 180.0 / M_PI,
            axial_error, force_mode_max_axial_deviation_, used_ratio * 100.0);
          force_mode_monitor_last_log_ = now;
        }
        if (std::fabs(axial_error) > force_mode_max_axial_deviation_) {
          RCLCPP_ERROR(this->get_logger(),
            "force-mode watchdog: axial deviation %.4fm exceeds %.4fm (relative_fz=%.3f N)",
            axial_error, force_mode_max_axial_deviation_, relative_fz);
          logForceDiagnostics("axial_deviation");
          abortPolishing("力控轴向偏差超过上限", true);
          return false;
        }
        return true;
      }

      void ysURForceAppControl::polish_doCurvePolishing() {
        if (sub_step_ == 404 ) 
        {
          control_dt_index_++;
          if (control_dt_index_!=control_dt_count_)  return;
          control_dt_index_=0;

          // 切向打磨开始计时，供过力退出诊断使用(每次打磨从起点开始时重置)。
          if (!polish_tangential_started_) {
            polish_tangential_started_ = true;
            polish_tangential_start_ = std::chrono::steady_clock::now();

            // 404 启动自检：确认接触偏移补偿是否生效。
            // start_axial_err 一开始就接近 0.03m → offset 没吃到名义轨迹；
            // 接近 0 且打磨中渐增 → 名义轨迹与板面存在角度偏差。
            const KDL::Frame nominal_start =
              frame_forceadjust_base_ * frame_polishcloud_transform_ * polishcurve_OriginFrames_[0];
            const KDL::Vector start_err = ys_curP_tcp_.p - nominal_start.p;
            const double start_axial_err =
              start_err.x() * force_mode_axis_base_.x() +
              start_err.y() * force_mode_axis_base_.y() +
              start_err.z() * force_mode_axis_base_.z();
            RCLCPP_INFO(this->get_logger(),
              "404 start self-check: force_adj_p=(%.4f,%.4f,%.4f) nominal_start=(%.4f,%.4f,%.4f) "
              "tcp=(%.4f,%.4f,%.4f) start_axial_err=%.4fm",
              frame_forceadjust_base_.p.x(), frame_forceadjust_base_.p.y(), frame_forceadjust_base_.p.z(),
              nominal_start.p.x(), nominal_start.p.y(), nominal_start.p.z(),
              ys_curP_tcp_.p.x(), ys_curP_tcp_.p.y(), ys_curP_tcp_.p.z(),
              start_axial_err);
          }
          
          //force adjust
          KDL::Frame  curFrame, tmpFrame, moveFrame;
          curFrame = ys_curP_tcp_;
          double k=0,dz;
          const double relative_fz = relFz();
          if (debug_skip_force_contact_ || (use_force_mode_ && force_mode_enabled_)) {
            k = 0;  // 调试空跑 / 控制器内建力控: 轨迹不叠加 z 力调整(z 由控制器闭环)
          } else if (fabs(relative_fz - target_fz_)<force_deadband_) {
            k=0;
          } else {
            k=fabs(relative_fz - target_fz_)/(relative_fz - target_fz_);
          }
          dz = k*adjust_dz_;
          if (dz>0) {
            dz = dz/2;
          }
          KDL::Frame offsetFrame;
          offsetFrame.p = curFrame.M*KDL::Vector(0,0,dz);
          offsetFrame.M = KDL::Rotation::RPY(0, 0, 0);
          frame_forceadjust_base_ = offsetFrame * frame_forceadjust_base_;

          // 2026-08-02 应用层慢姿态环: SDK旋转柔顺无阻尼、实机振荡啃边(-30N尖峰,已关闭),
          // 改在本层用20帧平均力矩对轨迹姿态做慢积分修正(绕TCP原点旋转, 不动位置)。
          // 只追砂盘安装角/贴合误差这类准静态偏差; 快扰动交给z向力控与watchdog。
          // 增益小+死区+±2°限幅+0.5°/s限速, 应用层百ms级延迟对该时间尺度无影响。
          // 符号约定: 接触点偏+x→My>0→需绕y负转抬起+x边, 故 d=-gain*M (Mx同理)。
          if (orient_adapt_enabled_ && force_mode_enabled_) {
            const double mx = ys_average_wrench_.torque.data[0];
            const double my = ys_average_wrench_.torque.data[1];
            const double dt = control_dt_count_ * 0.004;  // 控制周期(s)
            double d_rx = 0.0, d_ry = 0.0;
            if (std::fabs(mx) > orient_adapt_torque_deadband_) d_rx = -orient_adapt_gain_ * mx * dt;
            if (std::fabs(my) > orient_adapt_torque_deadband_) d_ry = -orient_adapt_gain_ * my * dt;
            const double max_step = orient_adapt_max_rate_ * dt;
            d_rx = std::clamp(d_rx, -max_step, max_step);
            d_ry = std::clamp(d_ry, -max_step, max_step);
            orient_adapt_rx_ = std::clamp(orient_adapt_rx_ + d_rx,
              -orient_adapt_max_angle_, orient_adapt_max_angle_);
            orient_adapt_ry_ = std::clamp(orient_adapt_ry_ + d_ry,
              -orient_adapt_max_angle_, orient_adapt_max_angle_);
          }
        
          // trajectory
          trajectory_msgs::msg::JointTrajectory ys_goal;
          ys_goal.header.stamp = this->now();
          ys_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              ys_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          //points
          KDL::JntArray ys_resultJnt(joint_size_);
          KDL::JntArray lastQ(joint_size_);
          lastQ = ys_cur_q_;
          rclcpp::Duration deltaT(0, 1E9/50);
          size_t i = (polishcurve_step_count_+sidepolish_step_count_)*polishcurve_yindex_
              +polishcurve_step_index_+sidepolish_step_index_;
          last_polish_step_ = static_cast<int>(i) + 1;
          // for (; i < polishcurve_OriginFrames_.size(); i++)
          {
            //ys_ik
            tmpFrame = frame_forceadjust_base_ * frame_polishcloud_transform_* polishcurve_OriginFrames_[i];
            if (orient_adapt_rx_ != 0.0 || orient_adapt_ry_ != 0.0) {
              // 慢姿态环输出: 绕TCP原点叠加旋转修正(仅姿态, 位置不变)
              tmpFrame.M = tmpFrame.M * KDL::Rotation::RPY(orient_adapt_rx_, orient_adapt_ry_, 0.0);
            }
            // tmpFrame = frame_forceadjust_base_ * frame_polishcloud_transform_*frame_polishcloud_base_ * polishcurve_OriginFrames_[i];
            if (!forceModeWatchdog(tmpFrame)) {
              return;
            }
            int rc = ys_tcp_tracik_solver_->CartToJnt(lastQ, tmpFrame, ys_resultJnt);
            if (rc == 1) {
              if (!KDL::Equal(lastQ, ys_resultJnt, M_PI/10)) {
                RCLCPP_FATAL(this->get_logger()," curve polish target %d lastQ, %f, %f, %f, %f, %f, %f", i+1,
                  lastQ(0)*180/M_PI, lastQ(1)*180/M_PI, lastQ(2)*180/M_PI, lastQ(3)*180/M_PI, lastQ(4)*180/M_PI, lastQ(5)*180/M_PI);
                RCLCPP_FATAL(this->get_logger()," curve polish target %d target, %f, %f, %f, %f, %f, %f", i+1,
                  ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
                ys_resultJnt = lastQ;
                // polish break
                RCLCPP_INFO(this->get_logger()," polish job break");
                markPolishFailure("打磨轨迹 IK 跳支");
                RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
                sub_step_++;
                disableForceMode();
                //init
                frame_forceadjust_base_.p = KDL::Vector(0,0,0);
                frame_forceadjust_base_.M = KDL::Rotation::RPY(0, 0, 0);
                orient_adapt_rx_ = 0.0;
                orient_adapt_ry_ = 0.0;
                sidepolish_step_index_=0;
                polishcurve_step_index_=0;
                polishcurve_yindex_=0;
                return;
              } else {
                // 每控制周期(40ms)打印目标关节角会刷屏, 注释掉(需要时开 DEBUG 级别)
                // RCLCPP_DEBUG(this->get_logger()," curve polish target %d target, %f, %f, %f, %f, %f, %f", i+1,
                //   ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
              }
              lastQ = ys_resultJnt;
            } else {
              RCLCPP_FATAL(this->get_logger()," curve polish target %d, trac ik failed", i+1);
              abortPolishing("打磨轨迹 IK 求解失败", true);
              return;
            }
            //traj point
            trajectory_msgs::msg::JointTrajectoryPoint tmpPt;
            for(int x=0;x<joint_size_;++x)
            {
                tmpPt.positions.push_back(ys_resultJnt(x));
                tmpPt.velocities.push_back(0);
            }
            tmpPt.time_from_start = deltaT;
            ys_goal.points.push_back(tmpPt);
          }

          //pub
          ys_traj_publisher_->publish(ys_goal);   

          // 2026-08-02 进给门控: 力控模式下轨迹索引原按控制周期无条件推进,
          // 力突增(尖峰/爬坡)时切向照走, 把打磨头"压着走"造成二次爬升(历次停摆直接死因)。
          // 现20帧平均力过载(feed_gate_fz)即暂停推进, 目标保持当前点, 等z向力控拉回带内;
          // 卡死超 feed_gate_timeout 按正常流程退出, 避免产线停摆悬死。
          bool feed_gated = false;
          double fz_avg_gate = 0.0;
          if (feed_gate_enabled_ && force_mode_enabled_) {
            fz_avg_gate = relFzAvg();
            feed_gated = fz_avg_gate <= feed_gate_fz_;
          }
          if (feed_gated) {
            if (feed_gate_hold_count_ == 0) {
              feed_gate_hold_start_ = std::chrono::steady_clock::now();
            }
            feed_gate_hold_count_++;
            if (feed_gate_hold_count_ % 25 == 1) {  // 约1s一条
              RCLCPP_WARN(this->get_logger(),
                "feed gated: fz_avg=%.2f N <= %.2f N, holding at step %d (%.1fs)",
                fz_avg_gate, feed_gate_fz_, last_polish_step_,
                std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - feed_gate_hold_start_).count());
            }
            if (std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - feed_gate_hold_start_).count()
                > feed_gate_timeout_) {
              RCLCPP_ERROR(this->get_logger(),
                "force-mode watchdog: feed gate stuck for %.1fs (fz_avg=%.2f N); aborting",
                feed_gate_timeout_, fz_avg_gate);
              logForceDiagnostics("feed_gate_timeout");
              abortPolishing("过力进给门控持续超时", true);
              return;
            }
          } else {
            feed_gate_hold_count_ = 0;
          }

          // 力控模式下 z 由控制器闭环调整, 实际位姿与名义路径点可能有偏差,
          // 不再用到位距离判断进度, 直接按控制周期推进(过载时由上门控暂停)。
          if (!feed_gated && (force_mode_enabled_ || (tmpFrame.p-curFrame.p).Norm()<0.001)) {
            //update index
            if (polishcurve_step_index_==polishcurve_step_count_){
              if (sidepolish_step_index_==sidepolish_step_count_){
                if (polishcurve_yindex_!=polishcurve_ycount_){
                  RCLCPP_INFO(this->get_logger(), "side index %d polish done. ", polishcurve_yindex_+1);
                  polishcurve_yindex_++;
                  sidepolish_step_index_=0;
                  polishcurve_step_index_=0;
                }
              } else {
                if (sidepolish_step_index_==0) {
                  RCLCPP_INFO(this->get_logger(), "curve index %d polish done. ", polishcurve_yindex_+1);
                }
                sidepolish_step_index_++;
                if (polishcurve_yindex_ == polishcurve_ycount_-1) {
                  // polish done
                  RCLCPP_INFO(this->get_logger()," polish job done");
                  RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
                  sub_step_++;
                  //init
                  frame_forceadjust_base_.p = KDL::Vector(0,0,0);
                  frame_forceadjust_base_.M = KDL::Rotation::RPY(0, 0, 0);
                  orient_adapt_rx_ = 0.0;
                  orient_adapt_ry_ = 0.0;
                  sidepolish_step_index_=0;
                  polishcurve_step_index_=0;
                  polishcurve_yindex_=0;
                }
              }
            } else {
              polishcurve_step_index_++;
            }
          } else {
            RCLCPP_INFO(this->get_logger()," polish job wait");
          }
        }
      }

      void ysURForceAppControl::goHome() {
        // 防御: 若力控仍开着(异常中止后回Home), 先关掉, 避免位置控制与力控打架
        disableForceMode();
        switch (sub_step_)
        {
        case 0://AppCommand::GO_HOME*100
          goHome_pubMoveHome();
          break;
        case 1:
          goHome_WaitMoveHomeDone();
          break;
        default:
          break;
        }

      }

      void ysURForceAppControl::goHome_pubMoveHome() {
        if (0==sub_step_) {
          // ys trajectory
          trajectory_msgs::msg::JointTrajectory ys_goal;
          ys_goal.header.stamp = this->now();
          ys_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              ys_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          RCLCPP_INFO(this->get_logger()," home  Q, %f, %f, %f, %f, %f, %f", 
          ys_home_q_(0)*180/M_PI, ys_home_q_(1)*180/M_PI, ys_home_q_(2)*180/M_PI, ys_home_q_(3)*180/M_PI, ys_home_q_(4)*180/M_PI, ys_home_q_(5)*180/M_PI);

          trajectory_msgs::msg::JointTrajectoryPoint ys_tmp_point;
          for(int i=0;i<joint_size_;++i)
          {
              ys_tmp_point.positions.push_back(ys_home_q_(i));
              ys_tmp_point.velocities.push_back(0);
          }
          ys_tmp_point.time_from_start.sec = 2*speed_level_;
          ys_tmp_point.time_from_start.nanosec =  1E9/125 ;  //0.008
          ys_goal.points.push_back(ys_tmp_point);

          //pub
          RCLCPP_INFO(this->get_logger(),"pub ys ur home trajectory");
          ys_traj_publisher_->publish(ys_goal);   
          // ysPolishTool_Open();

          sub_step_++;
        }
      }

      void ysURForceAppControl::goHome_WaitMoveHomeDone() {
        if (KDL::Equal(ys_cur_q_, ys_home_q_, joint_eps_) 
          &&sub_step_ == 1
        ) {
          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
          sub_step_ = 9999;
          const int32_t result_code = polish_cancelled_ ? 205
            : (polish_failed_ ? 204 : 100);
          if (result_code == 100) {
            std_msgs::msg::Int32 msg;
            msg.data = result_code;
            cmd_result_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(),
                "pub the result of command: %d. ", msg.data);
          } else {
            publishPolishResult(
              result_code,
              polish_result_detail_.empty()
                ? "打磨安全中止并已回 Home2" : polish_result_detail_);
          }
          app_cmd_ = AppCommand::NOTHING;
        }
      }

      void ysURForceAppControl::ysPolishTool_Open() {
        polish_tool_open_pending_ = false;
        polish_tool_open_done_ = false;
        polish_tool_open_ok_ = false;
        if (!polish_tool_io_client_ || !polish_tool_io_client_->service_is_ready()) {
          RCLCPP_ERROR(this->get_logger(), "SetIO service not available for tool open");
          polish_tool_open_done_ = true;
          return;
        }
        auto req = std::make_shared<eli_common_interface::srv::SetIO::Request>();
        req->fun = eli_common_interface::srv::SetIO::Request::FUN_SET_CONFIGURE_OUT;  // fun=2
        req->pin = 7;
        req->state = eli_common_interface::srv::SetIO::Request::STATE_ON;
        polish_tool_open_pending_ = true;
        polish_tool_open_request_start_ = std::chrono::steady_clock::now();
        polish_tool_io_client_->async_send_request(
          req,
          [this](rclcpp::Client<eli_common_interface::srv::SetIO>::SharedFuture future) {
            if (!polish_tool_open_pending_) {
              RCLCPP_WARN(this->get_logger(), "Ignoring stale polish tool open response");
              return;
            }
            try {
              polish_tool_open_ok_ = future.get()->success;
              if (polish_tool_open_ok_) {
                polish_tool_spinup_start_ = std::chrono::steady_clock::now();
                RCLCPP_INFO(this->get_logger(), "Polish tool open confirmed: fun=2 pin=7 state=true");
              } else {
                RCLCPP_ERROR(this->get_logger(), "Polish tool open rejected by SetIO service");
              }
            } catch (const std::exception & e) {
              polish_tool_open_ok_ = false;
              RCLCPP_ERROR(this->get_logger(), "Polish tool open SetIO exception: %s", e.what());
            }
            polish_tool_open_done_ = true;
          });
        RCLCPP_INFO(this->get_logger(), "Polish tool open command sent: fun=2 pin=7 state=true");
      }

      void ysURForceAppControl::ysPolishTool_Close() {
        // 关闭命令使尚未返回的开启响应失效，避免退出后污染状态。
        polish_tool_open_pending_ = false;
        polish_tool_open_done_ = false;
        polish_tool_open_ok_ = false;
        polish_tool_close_pending_ = false;
        polish_tool_close_done_ = false;
        polish_tool_close_ok_ = false;
        polish_tool_close_request_start_ = std::chrono::steady_clock::now();
        if (!polish_tool_io_client_ || !polish_tool_io_client_->service_is_ready()) {
          RCLCPP_ERROR(this->get_logger(), "SetIO service not available for tool close");
          polish_tool_close_done_ = true;
          return;
        }
        auto req = std::make_shared<eli_common_interface::srv::SetIO::Request>();
        req->fun = eli_common_interface::srv::SetIO::Request::FUN_SET_CONFIGURE_OUT;  // fun=2
        req->pin = 7;
        req->state = eli_common_interface::srv::SetIO::Request::STATE_OFF;
        polish_tool_close_pending_ = true;
        polish_tool_io_client_->async_send_request(
          req,
          [this](rclcpp::Client<eli_common_interface::srv::SetIO>::SharedFuture future) {
            if (!polish_tool_close_pending_) {
              RCLCPP_WARN(this->get_logger(), "Ignoring stale polish tool close response");
              return;
            }
            try {
              polish_tool_close_ok_ = future.get()->success;
              if (polish_tool_close_ok_) {
                RCLCPP_INFO(this->get_logger(), "Polish tool close confirmed: fun=2 pin=7 state=false");
              } else {
                RCLCPP_ERROR(this->get_logger(), "Polish tool close rejected by SetIO service");
              }
            } catch (const std::exception & e) {
              polish_tool_close_ok_ = false;
              RCLCPP_ERROR(this->get_logger(), "Polish tool close SetIO exception: %s", e.what());
            }
            polish_tool_close_pending_ = false;
            polish_tool_close_done_ = true;
          });
        RCLCPP_INFO(this->get_logger(), "Polish tool close command sent: fun=2 pin=7 state=false");
      }

    }
}
