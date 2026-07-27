#include "ysURForceAppControl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <list>
#include <chrono>
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
          speed_level_ = 2;
          initDataQ();

          RCLCPP_INFO(this->get_logger(),"init force data");
          //init force control
          target_fz_ = -8;//N
          adjust_dz_ = 0.0003;//m
          control_dt_count_ = 10;//todo, n*4ms for timer
          control_dt_index_ = 0;
          //polish data
          frame_polishcloud_base_.p = KDL::Vector(-0.11, 0.125, 0.205);
          frame_polishcloud_base_.M = KDL::Rotation::RPY(M_PI/2, 0, 0);
          frame_polishcloud_transform_.p = KDL::Vector(0,0,0);
          frame_polishcloud_transform_.M = KDL::Rotation::RPY(0, 0, 0);
          frame_forceadjust_base_.p = KDL::Vector(0,0,0);
          frame_forceadjust_base_.M = KDL::Rotation::RPY(0, 0, 0);
          polishcurve_radius_ = 0.9306;
          polishcurve_center_dz_ = 1.89539;
          polishcurve_start_ry_ = -9.2*M_PI/180;
          polishcurve_end_ry_ = 9.2*M_PI/180;
          polishproduct_width_ = 0.2;
          polishcurve_step_count_ = 500*speed_level_;
          polishcurve_step_index_ = 0;
          polishcurve_ycount_ = 1;
          polishcurve_yindex_ = 0;
          sidepolish_step_count_ = 100*speed_level_;
          sidepolish_step_index_ = 0;
          dz_polish_startup_tool_ = -0.08;//offset tool z for polish startup
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
          ys_contact_wrench_count = 200;
          ys_contact_wrench_arr.resize(ys_contact_wrench_count);
          ys_contact_wrench_index = 0;
          ys_average_wrench_.force = KDL::Vector(0,0,0);
          ys_average_wrench_.torque = KDL::Vector(0,0,0);
          ys_first_wrench_ = true;
          // 重力补偿参数（2026-07-26 负载识别，含打磨机+相机整体）:
          //   质量 1.478 kg，重力 = m*g*(世界系向下在 base 系的方向)（45°倾斜安装，
          //   方向向量 = -V_UP_IN_BASE = (0.7431,-0.0120,-0.6691)，见 biaoding/world_offset_calc.py）
          ys_tool_gravity_ = KDL::Vector(10.7654, -0.1738, -9.6917);
          //   质心在 tool0（法兰）系下的坐标（米），示教器读数 X=-56.32 Y=19.36 Z=-19.86 mm
          ys_tool_gcenter_ = KDL::Vector(-0.05632, 0.01936, -0.01986);
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
            // //tool
            // ys_polishtool_client_ = this->create_client<std_srvs::srv::SetBool>("ys_polishtool_setting"); //todo
            //timer
            motion_timer_ = this->create_wall_timer(
                4ms, std::bind(&ysURForceAppControl::timer_callback, this));
      }

      bool ysURForceAppControl::initDataQ(){
          ys_home_q_.resize(joint_size_);
          // 已示教（角度制，示教器读数）
          ys_home_q_.data << 8.2, -93.6, -110.2, 57.4, 91.7, 91.6;  
          for (int i=0;i<joint_size_;i++){
            ys_home_q_(i) = ys_home_q_(i)*M_PI/180;
          }
          ys_cameraCapture_q_.resize(joint_size_);
          // 已示教（角度制），与 home 同姿态
          ys_cameraCapture_q_.data <<  8.2, -93.6, -110.2, 57.4, 91.7, 91.6;
          for (int i=0;i<joint_size_;i++){
            ys_cameraCapture_q_(i) = ys_cameraCapture_q_(i)*M_PI/180;
          }
          ys_polishBase_q_.resize(joint_size_);
          // 已示教（弧度制，直接赋值，不要再做角度→弧度转换！）
          ys_polishBase_q_.data << 0.2583, -1.7438, -1.5621, 0.7505, 1.5043, 1.5330;
          return true;
      }

      void ysURForceAppControl::subCommandStateCB(const std_msgs::msg::Int32 msg) {
        int32_t cmd;
        cmd = msg.data;
        switch (cmd)
        {
        case 0:
          app_cmd_ = AppCommand::GO_HOME;
          sub_step_ = app_cmd_*100;
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
          app_cmd_ = AppCommand::DO_CAMERA_VISION_JOB;
          sub_step_ = app_cmd_*100;
          break;
        case 4:
          app_cmd_ = AppCommand::DO_CURVE_POLISHING;
          sub_step_ = app_cmd_*100;
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
          app_cmd_ = AppCommand::GO_HOME;
          sub_step_ = app_cmd_*100;
          return;
        }
        
        RCLCPP_INFO(this->get_logger(), "subVisionResultCB: type: %s: x %f ; y %f ; z %f", id.c_str(), result.pose.position.x, result.pose.position.y, result.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "subVisionResultCB: Quaternion: x %f ; y %f ; z %f; w %f", result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z, result.pose.orientation.w);
        if ( (sub_step_ == 303 && ys_vision_job_done_ == false) )
        {
          frame_polishcloud_transform_.p = KDL::Vector(result.pose.position.x,result.pose.position.y,result.pose.position.z);
          frame_polishcloud_transform_.M = KDL::Rotation::Quaternion(result.pose.orientation.x,result.pose.orientation.y,result.pose.orientation.z, result.pose.orientation.w);          
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
        KDL::Frame target;
        double start_ry = 0; 
        double end_ry = 0; 
        if (0 == yindex%2) {
          start_ry = polishcurve_start_ry_;
          end_ry = polishcurve_end_ry_;
        } else {
          start_ry = polishcurve_end_ry_;
          end_ry = polishcurve_start_ry_;
        }
        double ry = start_ry + stepindex *(end_ry - start_ry) / polishcurve_step_count_;
        target.p.data[0] = polishcurve_radius_ * sin(M_PI + ry);
        target.p.data[1] = polishproduct_width_/2-(0.5+yindex)*(polishproduct_width_/polishcurve_ycount_);
        target.p.data[2] = polishcurve_center_dz_ + polishcurve_radius_ * cos(M_PI + ry);
        target.M = KDL::Rotation::RPY(0, ry, 0);
        // RCLCPP_INFO(this->get_logger(), "Y %d polishcurve %d: x %f ; y %f ; z %f",yindex, stepindex, target.p.data[0], target.p.data[1], target.p.data[2]);

        return target;
      }
      KDL::Frame ysURForceAppControl::calcSidePolishPoint(int sideindex, int yindex){
        KDL::Frame target;
        double ry = 0; 
        if (0 == yindex%2) {
          ry = polishcurve_end_ry_;
        } else {
          ry = polishcurve_start_ry_;
        }
        target.p.data[0] = polishcurve_radius_ * sin(M_PI + ry);
        target.p.data[1] = polishproduct_width_/2-(0.5+yindex+(1.0*sideindex/sidepolish_step_count_))*(polishproduct_width_/polishcurve_ycount_);
        target.p.data[2] = polishcurve_center_dz_ + polishcurve_radius_ * cos(M_PI + ry);
        target.M = KDL::Rotation::RPY(0, ry, 0);
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
          if (rc == 1) {
            RCLCPP_INFO(this->get_logger()," polish start up target, %f, %f, %f, %f, %f, %f", 
              ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
          } else {
            RCLCPP_INFO(this->get_logger()," polish start up target, trac ik failed");
          }
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
      void ysURForceAppControl::polish_waitPolishBase() {
        if (sub_step_ == 401
        ) {
          //start up point
          KDL::Frame  startPos, upFrame;
          // startPos = frame_polishcloud_transform_* frame_polishcloud_base_ * polishcurve_OriginFrames_[0];
          startPos = frame_polishcloud_transform_*  polishcurve_OriginFrames_[0];
          //ik
          KDL::Frame moveFrame;
          moveFrame.p = KDL::Vector(0,0,dz_polish_startup_tool_);//offset tool z 
          moveFrame.M = KDL::Rotation::RPY(0,0,0);
          upFrame = startPos * moveFrame;
          KDL::JntArray ys_resultJnt(joint_size_);
          int rc = ys_tcp_tracik_solver_->CartToJnt(ys_polishBase_q_, upFrame, ys_resultJnt);
          if (rc != 1) {
            RCLCPP_INFO(this->get_logger()," polish start up target, trac ik failed");
          }
          if (KDL::Equal(ys_cur_q_, ys_resultJnt, joint_eps_)
          ) {
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_++;
          }
        }
      }
      void ysURForceAppControl::polish_doForceContact() {
        if (sub_step_ == 402 ) 
        {
          bool touch = false;

          //start up point
          KDL::Frame  startPos, upFrame;
          startPos = frame_polishcloud_transform_ *  polishcurve_OriginFrames_[0];
          // startPos = frame_polishcloud_transform_ * frame_polishcloud_base_ * polishcurve_OriginFrames_[0];
          //ik
          KDL::Frame  curFrame, tmpFrame, moveFrame;
          moveFrame.p = KDL::Vector(0,0,dz_polish_startup_tool_);//offset tool z 
          moveFrame.M = KDL::Rotation::RPY(0,0,0);
          upFrame = startPos * moveFrame;
          curFrame = ys_curP_tcp_;
          curFrame.M = startPos.M;

          double dcontact = 0.01;
          if ( ys_average_wrench_.force.data[2] < target_fz_*2)
          // if ( std::sqrt((curFrame.p.data[0]-upFrame.p.data[0])*(curFrame.p.data[0]-upFrame.p.data[0])
          //       +(curFrame.p.data[1]-upFrame.p.data[1])*(curFrame.p.data[1]-upFrame.p.data[1])
          //       +(curFrame.p.data[2]-upFrame.p.data[2])*(curFrame.p.data[2]-upFrame.p.data[2])
          //     )>fabs(dz_polish_startup_tool_))
          {
            dcontact = -0.001;
            touch = true;
            RCLCPP_INFO(this->get_logger()," contact delta d %f ", dcontact);
          }
          control_dt_index_++;
          if (control_dt_index_!=control_dt_count_&&touch==false)  return;
          control_dt_index_=0;
          
          // trajectory
          trajectory_msgs::msg::JointTrajectory ys_goal;
          ys_goal.header.stamp = this->now();
          ys_goal.header.frame_id = "new";
          for(int i=0;i<joint_size_;++i)      {
              ys_goal.joint_names.push_back(ys_prefix_ + joint_names_[i]);
          }
          
          //fk
          KDL::JntArray ys_resultJnt(joint_size_);
          KDL::JntArray lastQ(joint_size_);
          lastQ = ys_cur_q_;
          rclcpp::Duration deltaT(0, 1E9/125);

          int trajCount=30*speed_level_;
          for (size_t i = 0; i < trajCount; i++)
          {
            //ys_ik
            moveFrame.p = KDL::Vector(0,0,dcontact)*(i+1)/trajCount;//offset tool z 
            moveFrame.M = KDL::Rotation::RPY(0,0,0);
            // RCLCPP_INFO(this->get_logger(), "moveFrame: x %f ; y %f ; z %f", moveFrame.p.data[0], moveFrame.p.data[1], moveFrame.p.data[2]);
            tmpFrame = curFrame*moveFrame;
            int rc = ys_tcp_tracik_solver_->CartToJnt(lastQ, tmpFrame, ys_resultJnt);
            if (rc == 1) {
              if (!KDL::Equal(lastQ, ys_resultJnt, M_PI/10)) {
                RCLCPP_FATAL(this->get_logger()," ys_resultJnt  lastQ, %f, %f, %f, %f, %f, %f", 
                  lastQ(0)*180/M_PI, lastQ(1)*180/M_PI, lastQ(2)*180/M_PI, lastQ(3)*180/M_PI, lastQ(4)*180/M_PI, lastQ(5)*180/M_PI);
                RCLCPP_FATAL(this->get_logger()," ys_resultJnt  target, %f, %f, %f, %f, %f, %f", 
                  ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
                ys_resultJnt = lastQ;
              }
              lastQ = ys_resultJnt;
            } else {
              RCLCPP_INFO(this->get_logger()," contact target %d, trac ik failed", i+1);
            }
            //traj point
            trajectory_msgs::msg::JointTrajectoryPoint tmpPt;
            for(int x=0;x<joint_size_;++x)
            {
                tmpPt.positions.push_back(ys_resultJnt(x));
                tmpPt.velocities.push_back(0);
            }
            tmpPt.time_from_start = rclcpp::Duration::from_nanoseconds(deltaT.nanoseconds() * (i + 1));// 时间必须逐点递增，否则被控制器拒绝
            ys_goal.points.push_back(tmpPt);
          }

          //pub
          // RCLCPP_INFO(this->get_logger(),"pub contact start polish trajectory");
          ys_traj_publisher_->publish(ys_goal);   

          if ( ys_average_wrench_.force.data[2] < target_fz_*2)
          // if ( std::sqrt((curFrame.p.data[0]-upFrame.p.data[0])*(curFrame.p.data[0]-upFrame.p.data[0])
          //       +(curFrame.p.data[1]-upFrame.p.data[1])*(curFrame.p.data[1]-upFrame.p.data[1])
          //       +(curFrame.p.data[2]-upFrame.p.data[2])*(curFrame.p.data[2]-upFrame.p.data[2])
          //     )>fabs(dz_polish_startup_tool_))
          {
            RCLCPP_INFO(this->get_logger()," contact  polish start point");
            RCLCPP_FATAL(this->get_logger()," ys_resultJnt  lastQ, %f, %f, %f, %f, %f, %f", 
            lastQ(0)*180/M_PI, lastQ(1)*180/M_PI, lastQ(2)*180/M_PI, lastQ(3)*180/M_PI, lastQ(4)*180/M_PI, lastQ(5)*180/M_PI);
            RCLCPP_INFO(this->get_logger()," ys_resultJnt  curQ, %f, %f, %f, %f, %f, %f", 
            ys_cur_q_(0)*180/M_PI, ys_cur_q_(1)*180/M_PI, ys_cur_q_(2)*180/M_PI, ys_cur_q_(3)*180/M_PI, ys_cur_q_(4)*180/M_PI, ys_cur_q_(5)*180/M_PI);
            //todo calcOffsetFrame
            KDL::Frame offsetFrame;
            offsetFrame.p = curFrame.p - startPos.p;
            offsetFrame.M = KDL::Rotation::RPY(0, 0, 0);
            RCLCPP_INFO(this->get_logger()," ys offsetFrame, x: %f, y: %f, z: %f", 
            offsetFrame.p.data[0], offsetFrame.p.data[1], offsetFrame.p.data[2]);
            frame_forceadjust_base_ = offsetFrame * frame_forceadjust_base_;
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_++;
          }
        }
      }
      void ysURForceAppControl::polish_startPolishtool() {
        if (sub_step_ == 403
        ) {
            ysPolishTool_Open();
            RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_++;
        }
      }
      void ysURForceAppControl::polish_endPolishtool() {
        if (sub_step_ == 405
        ) {
          ysPolishTool_Close();
          RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
            sub_step_++;
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
          // 45° 倾斜安装: 退刀按"世界系竖直向上"抬刀（实测向量，与 biaoding/world_offset_calc.py
          // 的 V_UP_IN_BASE 一致），同时 base y 方向侧移 0.1m（base y 仍接近世界水平）。
          const KDL::Vector world_up_in_base(-0.7431, 0.0120, 0.6691);
          const double lift_height = 0.15;  // 世界系竖直抬刀高度(米)，按现场调整
          for (size_t i = 0; i < trajCount; i++)
          {
            //ys_ik
            lastQ = ys_resultJnt;
            moveFrame.p = KDL::Vector(0,0.1*(i+1)/trajCount,0) + world_up_in_base*(lift_height*(i+1)/trajCount);//base y侧移 + 世界系竖直抬刀
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
              }
            } else {
              RCLCPP_INFO(this->get_logger()," back polish end up target %d, trac ik failed", i+1);
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
          std_msgs::msg::Int32 msg;
          msg.data = 104;//100+ polish done
          cmd_result_publisher_->publish(msg);
          app_cmd_ = AppCommand::NOTHING;
          // app_cmd_ = AppCommand::DO_AGV_GO_HOME;  // Elite: no AGV
          sub_step_ = app_cmd_*100;
          RCLCPP_INFO(this->get_logger(),
              "pub the result of command: %d. ", msg.data);
        }
      }

      void ysURForceAppControl::polish_doCurvePolishing() {
        if (sub_step_ == 404 ) 
        {
          control_dt_index_++;
          if (control_dt_index_!=control_dt_count_)  return;
          control_dt_index_=0;
          
          //force adjust
          KDL::Frame  curFrame, tmpFrame, moveFrame;
          curFrame = ys_curP_tcp_;
          double k=0,dz;
          if (fabs(ys_contact_wrench_sensor_.force.data[2] - target_fz_)<3) {
            k=0;
          } else {
            k=fabs(ys_contact_wrench_sensor_.force.data[2] - target_fz_)/(ys_contact_wrench_sensor_.force.data[2] - target_fz_);
          }
          dz = k*adjust_dz_;
          if (dz>0) {
            dz = dz/2;
          }
          KDL::Frame offsetFrame;
          offsetFrame.p = curFrame.M*KDL::Vector(0,0,dz);
          offsetFrame.M = KDL::Rotation::RPY(0, 0, 0);
          frame_forceadjust_base_ = offsetFrame * frame_forceadjust_base_;
        
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
          // for (; i < polishcurve_OriginFrames_.size(); i++)
          {
            //ys_ik
            tmpFrame = frame_forceadjust_base_ * frame_polishcloud_transform_* polishcurve_OriginFrames_[i];
            // tmpFrame = frame_forceadjust_base_ * frame_polishcloud_transform_*frame_polishcloud_base_ * polishcurve_OriginFrames_[i];
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
                RCLCPP_INFO(this->get_logger(), "sub step done: %d. ", sub_step_);
                sub_step_++;
                //init
                frame_forceadjust_base_.p = KDL::Vector(0,0,0);
                frame_forceadjust_base_.M = KDL::Rotation::RPY(0, 0, 0);
                sidepolish_step_index_=0;
                polishcurve_step_index_=0;
                polishcurve_yindex_=0;
                return;
              } else {
                RCLCPP_INFO(this->get_logger()," curve polish target %d target, %f, %f, %f, %f, %f, %f", i+1,
                  ys_resultJnt(0)*180/M_PI, ys_resultJnt(1)*180/M_PI, ys_resultJnt(2)*180/M_PI, ys_resultJnt(3)*180/M_PI, ys_resultJnt(4)*180/M_PI, ys_resultJnt(5)*180/M_PI);
              }
              lastQ = ys_resultJnt;
            } else {
              RCLCPP_FATAL(this->get_logger()," curve polish target %d, trac ik failed", i+1);
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

          if ((tmpFrame.p-curFrame.p).Norm()<0.001) {
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
          std_msgs::msg::Int32 msg;
          msg.data = 100;//go home done
          cmd_result_publisher_->publish(msg);
          app_cmd_ = AppCommand::NOTHING;
          RCLCPP_INFO(this->get_logger(),
              "pub the result of command: %d. ", msg.data);
        }
      }

      void ysURForceAppControl::ysPolishTool_Open() {
        auto node = rclcpp::Node::make_shared("_polish_tool_client");
        auto client = node->create_client<eli_common_interface::srv::SetIO>(
          "/io_and_status_controller/set_io");
        if (!client->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_ERROR(this->get_logger(), "SetIO service not available for tool open");
          return;
        }
        auto req = std::make_shared<eli_common_interface::srv::SetIO::Request>();
        req->fun = eli_common_interface::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
        req->pin = 0;
        req->state = 1.0;  // ON
        client->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Polish tool open command sent");
      }

      void ysURForceAppControl::ysPolishTool_Close() {
        auto node = rclcpp::Node::make_shared("_polish_tool_client");
        auto client = node->create_client<eli_common_interface::srv::SetIO>(
          "/io_and_status_controller/set_io");
        if (!client->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_ERROR(this->get_logger(), "SetIO service not available for tool close");
          return;
        }
        auto req = std::make_shared<eli_common_interface::srv::SetIO::Request>();
        req->fun = eli_common_interface::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
        req->pin = 0;
        req->state = 0.0;  // OFF
        client->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Polish tool close command sent");
      }

    }
}


