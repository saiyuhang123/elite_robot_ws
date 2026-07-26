#include "ysCamera3DSolver.hpp"
#include "pclTemplateAlign.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <list>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <tf2_eigen_kdl/tf2_eigen_kdl.hpp>
#include "pclCalcTransform.hpp"

namespace elite_robot {
    namespace ur_force_app {
      using namespace std::chrono_literals;
      using std::placeholders::_1;

      ysCamera3DSolver::ysCamera3DSolver() 
              : Node("ysCamera3DSolver") 
      {
        template_pathname_ = this->declare_parameter<std::string>("template_file", "etc/polish_feature_template0.5.pcd");
        this->get_parameter<std::string>("template_file", template_pathname_);
        RCLCPP_INFO(rclcpp::get_logger("ysCamera3DSolver"), "template_file: '%s'.", template_pathname_.c_str());

        //init app
          app_cmd_ = -1;
          //init  robot
          RCLCPP_INFO(this->get_logger(),"init robot data");
          joint_size_ = 6;
          joint_names_.push_back("shoulder_pan_joint");
          joint_names_.push_back("shoulder_lift_joint");
          joint_names_.push_back("elbow_joint");
          joint_names_.push_back("wrist_1_joint");
          joint_names_.push_back("wrist_2_joint");
          joint_names_.push_back("wrist_3_joint");
          ys_prefix_ = "cs66_";

          ys_cur_q_.resize(joint_size_);
          for (int i = 0; i < joint_size_; ++i) {
              ys_cur_q_(i) = 0;
          }
          ys_first_q_ = true;

          //eye
          try_count_ = 0;

          //fk ik
          ys_eye_fk_solver_ = nullptr;

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
              //ys eye
              // 眼在手上相机: 链只到 tool0（URDF 中无 tr_camera link），
              // tool0->相机光学系的固定变换在点云回调中乘上（见下方 ys_T_tool0_camera_）。
                std::string ys_base = ys_prefix_ + "base";
                std::string ys_tip_link = ys_prefix_ + "tool0";
                if (kdl_tree.getChain(ys_base, ys_tip_link, ys_eye_chain_)) {
                  ys_eye_fk_solver_ = new KDL::ChainFkSolverPos_recursive(ys_eye_chain_);
                } else {
                  RCLCPP_FATAL(this->get_logger(),"Couldn't find chain %s to %s", ys_base.c_str(), ys_tip_link.c_str());
                }
                //ik eye
                double timeout=0.005;
                double joint_eps = 1e-5;
                ys_eye_tracik_solver_ = new TRAC_IK::TRAC_IK(ys_base, ys_tip_link, xml_string, timeout, joint_eps);
            } else {
                RCLCPP_FATAL(this->get_logger(),"Failed to extract kdl tree from xml robot description");
            }
          }

            RCLCPP_INFO(this->get_logger(),"init publisher and subscriber data");
            cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(       
              "/elite_vision_job_cmd", 1, std::bind(&ysCamera3DSolver::subCommandStateCB, this, _1)); 
            ys_jointstates_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(       
                "/joint_states", 1, std::bind(&ysCamera3DSolver::ys_subJointStateCB, this, _1)); 
            ys_pointcloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(       
                "/camera/camera/depth/color/points", 1, std::bind(&ysCamera3DSolver::ys_subEyePointCloudCB, this, _1)); 
            ys_polish_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("elite_vision_pose_broadcaster/pose", 1); 
      }

      void ysCamera3DSolver::subCommandStateCB(const std_msgs::msg::Int32 msg) {
        int32_t cmd;
        cmd = msg.data;
        switch (cmd)
        {
        case 1:
          app_cmd_ = EyeCommand::DO_CAMERA_VISION_JOB;
          break;
        default:
          app_cmd_ = EyeCommand::NOTHING;
        }

        RCLCPP_INFO(this->get_logger(), "subCommandStateCB: %d. ", app_cmd_);
      }

      void ysCamera3DSolver::ys_subJointStateCB(const sensor_msgs::msg::JointState state) {
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

      void ysCamera3DSolver::ys_subEyePointCloudCB(const sensor_msgs::msg::PointCloud2 msg) {
        if (app_cmd_==EyeCommand::DO_CAMERA_VISION_JOB) {
          if (try_count_>5) {
            //geometry_msgs::msg::PoseStamped
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "failed";
            msg.pose.position.x = 0;
            msg.pose.position.y = 0;
            msg.pose.position.z = 0;
            msg.pose.orientation.x = 0;
            msg.pose.orientation.y = 0;
            msg.pose.orientation.z = 0;
            msg.pose.orientation.w = 1;
            RCLCPP_INFO(this->get_logger(), "failed to camera capture position:");
            ys_polish_pose_publisher_->publish(msg);

            try_count_=0;
            app_cmd_ = EyeCommand::NOTHING;
            return;
          }

          // RCLCPP_INFO(this->get_logger(),"sub ys eye point cloud size: %d", msg.);
          pcl::PCLPointCloud2 pcl_pc2;
          pcl_conversions::toPCL(msg,pcl_pc2);
          eye_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromPCLPointCloud2(pcl_pc2,*eye_cloud_);
          if (eye_cloud_->size()<250000)  return;

          if (ys_first_q_==false&&ys_eye_fk_solver_!=nullptr)
          {
            ys_eye_fk_solver_->JntToCart(ys_cur_q_, ys_curP_eye_);
            // 眼在手上: tool0 -> 相机光学系的固定变换（手眼标定 biaoding/hand_eye_result.json，
            // cv2.calibrateHandEye 输出的 R_cam2tool / t_cam2tool；RPY 由 R_cam2tool 换算）。
            // KDL 链只到 tool0，相机位姿 = tool0位姿 * 该固定变换。
            static const KDL::Frame ys_T_tool0_camera_(
                KDL::Rotation::RPY(-0.063913, 0.009927, -1.552858),
                KDL::Vector(-0.135488, 0.025233, 0.033736));
            ys_curP_eye_ = ys_curP_eye_ * ys_T_tool0_camera_;
            Eigen::Affine3d transform;
            tf2::transformKDLToEigen(ys_curP_eye_, transform);
            base_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud (*eye_cloud_, *base_cloud_, transform);
            pcl::io::savePCDFileBinary (template_pathname_+"cameracapture.pcd", *eye_cloud_);
            pcl::io::savePCDFileBinary (template_pathname_+"base.pcd", *base_cloud_);
            if ( ys_doPCLTemplateAlign() ){
              RCLCPP_INFO(this->get_logger(),"pcl cloud aligin successed.");
              app_cmd_ = EyeCommand::NOTHING;
              try_count_ = 0;
            } else {
              try_count_++;
            }
          }
        }
      }

      bool ysCamera3DSolver::ys_doPCLTemplateAlign() {
        bool ret = true;
  
        RCLCPP_INFO(this->get_logger(), "camera capture cloud size %d\n", base_cloud_->size());
        YsPCLCalcTransform calcFrame;
        Eigen::Matrix4f result;
        calcFrame.calc(base_cloud_, result);
        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotationC = result.block<3,3>(0, 0);
        Eigen::Vector3f translationC = result.block<3,1>(0, 3);
      
        printf ("\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", rotationC (0,0), rotationC (0,1), rotationC (0,2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", rotationC (1,0), rotationC (1,1), rotationC (1,2));
        printf ("    | %6.3f %6.3f %6.3f | \n", rotationC (2,0), rotationC (2,1), rotationC (2,2));
        printf ("\n");
        printf ("t = < %0.3f, %0.3f, %0.3f >\n", translationC (0), translationC (1), translationC (2));
    
        // // Preprocess the cloud by...
        // // ...removing distant points
        // pcl::PointCloud<pcl::PointXYZ>::Ptr distCloud (new pcl::PointCloud<pcl::PointXYZ>); 
        // const float depth_limit = -1.2;
        // pcl::PassThrough<pcl::PointXYZ> zpass;
        // zpass.setInputCloud (base_cloud_);
        // zpass.setFilterFieldName ("y");
        // zpass.setFilterLimits ( depth_limit, 0.6);
        // zpass.filter (*distCloud);
        // RCLCPP_INFO(this->get_logger(),"pass z 1.0 cloud size %d\n", distCloud->size());
        
        // pcl::PassThrough<pcl::PointXYZ> xpass;
        // xpass.setInputCloud (distCloud);
        // xpass.setFilterFieldName ("x");
        // xpass.setFilterLimits (-0.6, 0.2);
        // xpass.filter (*distCloud);
        // RCLCPP_INFO(this->get_logger(),"pass x 0.2 cloud size %d\n", distCloud->size());
        // pcl::io::savePCDFileBinary (template_pathname_+"target.pcd", *distCloud);
      
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temcloud (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::io::loadPCDFile (template_pathname_, *temcloud);
        // RCLCPP_INFO(this->get_logger(),"load template size %d\n", temcloud->size());
        // //downsampling the point cloud
        // const float voxel_grid_size = 0.005f;
        // pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        // vox_grid.setInputCloud (temcloud);
        // vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>); 
        // vox_grid.filter (*tmpCloud);
        // RCLCPP_INFO(this->get_logger(),"VoxelGrid 0.005 template size %d\n", tmpCloud->size());
        // // Assign to the template FeatureCloud
        // FeatureCloud template_cloud;
        // template_cloud.setInputCloud (tmpCloud);
      
        // // downsampling the point cloud
        // vox_grid.setInputCloud (distCloud);
        // vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr voxcloud (new pcl::PointCloud<pcl::PointXYZ>); 
        // vox_grid.filter (*voxcloud);
        // RCLCPP_INFO(this->get_logger(),"VoxelGrid 0.005 cameracapture size %d\n", voxcloud->size());
        // // Assign to the target FeatureCloud
        // FeatureCloud target_cloud;
        // target_cloud.setInputCloud (voxcloud);
      
        // // Set the TemplateAlignment inputs
        // TemplateAlignment template_align;
        // // template_align.addTemplateCloud (template_cloud);
        // template_align.setTargetCloud (target_cloud);
      
        // // Find the best template alignment
        // TemplateAlignment::Result best_alignment;
        // template_align.align(template_cloud, best_alignment);
      
        // // Print the alignment fitness score (values less than 0.00002 are good)
        // RCLCPP_INFO(this->get_logger(),"Best fitness score: %f\n", best_alignment.fitness_score);
      
        // // Print the rotation matrix and translation vector
        // Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
        // Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
      
        // RCLCPP_INFO(this->get_logger(),"\n");
        // RCLCPP_INFO(this->get_logger(),"    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        // RCLCPP_INFO(this->get_logger(),"R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        // RCLCPP_INFO(this->get_logger(),"    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        // RCLCPP_INFO(this->get_logger(),"\n");
        // RCLCPP_INFO(this->get_logger(),"t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
      
        // // Save the aligned template for visualization
        // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        // pcl::transformPointCloud (*template_cloud.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
        // pcl::io::savePCDFileBinary (template_pathname_+"result.pcd", transformed_cloud);
        // RCLCPP_INFO(this->get_logger(),"save  result.pcd\n");

        //geometry_msgs::msg::PoseStamped
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "tr_camera";
        msg.pose.position.x = translationC (0);
        msg.pose.position.y = translationC (1);
        msg.pose.position.z = translationC (2);
        Eigen::Quaternionf quaternion(rotationC);
        msg.pose.orientation.x = quaternion.x();
        msg.pose.orientation.y = quaternion.y();
        msg.pose.orientation.z = quaternion.z();
        msg.pose.orientation.w = quaternion.w();
        RCLCPP_INFO(this->get_logger(), "camera capture position: x %f ; y %f ; z %f", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "Quaternion: x %f ; y %f ; z %f; w %f", msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w);
        //check result
        // if (best_alignment.fitness_score>0.000015
        // || std::fabs(translation(0)) > 0.12
        // || std::fabs(translation(1)) > 0.12
        // || std::fabs(translation(2)) > 0.12
        // ) {
        //   ret = false;
        // } else {
          ys_polish_pose_publisher_->publish(msg);
          ret = true;
        // }
        return ret;
      }

    }
}


