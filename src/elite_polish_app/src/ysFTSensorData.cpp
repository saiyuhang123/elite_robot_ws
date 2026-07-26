#include "ysFTSensorData.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <list>
#include <chrono>

namespace elite_robot {
    namespace ur_force_app {
      using namespace std::chrono_literals;
      using std::placeholders::_1;

      ysFTSensorData::ysFTSensorData() 
              : Node("ysFTSensorData") 
      {
          //init  robot
          RCLCPP_INFO(this->get_logger(),"init robot data");
          joint_size_ = 6;
          joint_names_.push_back("shoulder_pan_joint");
          joint_names_.push_back("shoulder_lift_joint");
          joint_names_.push_back("elbow_joint");
          joint_names_.push_back("wrist_1_joint");
          joint_names_.push_back("wrist_2_joint");
          joint_names_.push_back("wrist_3_joint");
          ys_prefix_ = "ys_";

          ys_cur_q_.resize(joint_size_);
          for (int i = 0; i < joint_size_; ++i) {
              ys_cur_q_(i) = 0;
          }
          ys_first_q_ = true;

          RCLCPP_INFO(this->get_logger(),"init force torque data");
          //ftsensor
          ys_contact_wrench_sensor_.force = KDL::Vector(0,0,0);
          ys_contact_wrench_sensor_.torque = KDL::Vector(0,0,0);
          ys_wrench_count = 200;
          ys_wrench_base_arr.resize(ys_wrench_count);
          ys_wrench_index_ = 0;
          ys_first_wrench_ = true;
          ys_tool_gravity_ = KDL::Vector(-0.05, 0.19, -24.29);
          ys_tool_gcenter_ = KDL::Vector(0.025, -0.034, 0.046);
          ys_bias_wrench_.force = KDL::Vector(-1.02, -3.25, 0.98);
          ys_bias_wrench_.torque = KDL::Vector(-0.123, 0.060, 0.25);

          //fk ik
          ys_ftsensor_fk_solver_ = NULL;

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
          RCLCPP_INFO(this->get_logger(),"xml_string: %s", xml_string.c_str());
          
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
              //ys_ftsensor
                std::string ys_base = ys_prefix_ + "base";
                std::string ys_tip_link = ys_prefix_ + "kwr75";
                if (kdl_tree.getChain(ys_base, ys_tip_link, ys_ftsensor_chain_)) {
                  ys_ftsensor_fk_solver_ = new KDL::ChainFkSolverPos_recursive(ys_ftsensor_chain_);
                } else {
                  RCLCPP_FATAL(this->get_logger(),"Couldn't find chain %s to %s", ys_base.c_str(), ys_tip_link.c_str());
                }
            } else {
                RCLCPP_FATAL(this->get_logger(),"Failed to extract kdl tree from xml robot description");
            }
          }

            RCLCPP_INFO(this->get_logger(),"init publisher and subscriber data");
            ys_jointstates_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(       
                "/joint_states", 1, std::bind(&ysFTSensorData::ys_subJointStateCB, this, _1)); 
            ys_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(       
                "/ysrob_kw75/ysrob_fts_broadcaster/wrench", 1, std::bind(&ysFTSensorData::ys_subWrenchCB, this, _1)); 
            ys_contact_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("ys_contact_fts_broadcaster/wrench", 1); 
      }

      void ysFTSensorData::ys_subJointStateCB(const sensor_msgs::msg::JointState state) {
          // RCLCPP_INFO(this->get_logger(),"sub ys ur state, %f, %f, %f, %f, %f, %f", 
            // state.position[0]*180/M_PI,state.position[1]*180/M_PI,state.position[2]*180/M_PI,state.position[3]*180/M_PI,state.position[4]*180/M_PI, state.position[5]*180/M_PI);
          //get joint array
          KDL::JntArray jntArr;
          jntArr.resize(joint_size_);
          int n = state.name.size();
          for (int i = 0; i < joint_size_; ++i)//joint_names_
          {
              int x = 0;
              for (; x < n; ++x)//state
              {
                  if (state.name[x] == (ys_prefix_ + joint_names_[i])) {
                      jntArr(i) = state.position[x];
                      break;
                  }
              }

              if (x == n) {
                  return;
              }
          }
          ys_cur_q_ = jntArr;

          ys_first_q_ = false;
      }

      void ysFTSensorData::ys_subWrenchCB(const geometry_msgs::msg::WrenchStamped msg) {
          // RCLCPP_INFO(this->get_logger(),"sub ys wrench: force, x: %f, y: %f, z: %f", msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
          KDL::Wrench data,sum;
          data.force.data[0] = msg.wrench.force.x;
          data.force.data[1] = msg.wrench.force.y;
          data.force.data[2] = msg.wrench.force.z;
          data.torque.data[0] = msg.wrench.torque.x;
          data.torque.data[1] = msg.wrench.torque.y;
          data.torque.data[2] = msg.wrench.torque.z;

          //fix bias data
          if (ys_first_q_ == false
            && ys_first_wrench_ == true
            && ys_ftsensor_fk_solver_
          ) {
            KDL::Wrench biasdata = ys_gravityRepairWrench(data);
            ys_wrench_base_arr[ys_wrench_index_] = biasdata;
            ys_wrench_index_++;

            if (ys_wrench_index_ == ys_wrench_count)
            {
              sum.force = KDL::Vector(0,0,0);
              sum.torque = KDL::Vector(0,0,0);
              for (size_t i = 0; i < ys_wrench_count; i++)
              {
                sum += ys_wrench_base_arr[i];
              }
              RCLCPP_INFO(this->get_logger(),"init bias wrench: force, x: %f, y: %f, z: %f", ys_bias_wrench_.force.data[0], ys_bias_wrench_.force.data[1], ys_bias_wrench_.force.data[2]);
              ys_bias_wrench_ = sum / ys_wrench_count;
              RCLCPP_INFO(this->get_logger(),"device bias wrench: force, x: %f, y: %f, z: %f", ys_bias_wrench_.force.data[0], ys_bias_wrench_.force.data[1], ys_bias_wrench_.force.data[2]);

              ys_wrench_index_ = 0;
              ys_first_wrench_ = false;
            }
          }

          //calc contact wrench data
          if (ys_first_wrench_==false) {
            ys_contact_wrench_sensor_ = ys_gravityRepairWrench(data) - ys_bias_wrench_;

            // ys contact wrench pub
            geometry_msgs::msg::WrenchStamped msg_contact_fts;
            msg_contact_fts.header.stamp = this->now();
            msg_contact_fts.header.frame_id = "ys_kwr75";
            msg_contact_fts.wrench.force.x = ys_contact_wrench_sensor_.force.data[0];
            msg_contact_fts.wrench.force.y = ys_contact_wrench_sensor_.force.data[1];
            msg_contact_fts.wrench.force.z = ys_contact_wrench_sensor_.force.data[2];
            msg_contact_fts.wrench.torque.x = ys_contact_wrench_sensor_.torque.data[0];
            msg_contact_fts.wrench.torque.y = ys_contact_wrench_sensor_.torque.data[1];
            msg_contact_fts.wrench.torque.z = ys_contact_wrench_sensor_.torque.data[2];

            ys_contact_wrench_publisher_->publish(msg_contact_fts);   
            
          }
      }

      KDL::Wrench ysFTSensorData::ys_gravityRepairWrench(const KDL::Wrench &data) {
        KDL::Wrench value;
        //fk
        if (ys_ftsensor_fk_solver_) {
          ys_ftsensor_fk_solver_->JntToCart(ys_cur_q_, ys_curP_ftsensor_);
          KDL::Vector gravity_sensor = ys_curP_ftsensor_.M.Inverse()*ys_tool_gravity_;
          value.force = data.force - gravity_sensor;
          value.torque.data[0] = data.torque.data[0] 
              - (gravity_sensor.data[2]*ys_tool_gcenter_.data[1] - gravity_sensor.data[1]*ys_tool_gcenter_.data[2]);
          value.torque.data[1] = data.torque.data[0] 
              - (gravity_sensor.data[0]*ys_tool_gcenter_.data[2] - gravity_sensor.data[2]*ys_tool_gcenter_.data[0]);
          value.torque.data[2] = data.torque.data[0] 
              - (gravity_sensor.data[1]*ys_tool_gcenter_.data[0] - gravity_sensor.data[0]*ys_tool_gcenter_.data[1]);
        }
        return value;
      }

    }
}


