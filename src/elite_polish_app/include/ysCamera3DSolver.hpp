#ifndef ysCamera3DSolver_HPP
#define ysCamera3DSolver_HPP

#include <vector>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pclTemplateAlign.hpp"

namespace elite_robot {
    namespace ur_force_app {

        class ysCamera3DSolver  : public rclcpp::Node
        {
            enum EyeCommand {
                DO_CAMERA_VISION_JOB = 1,
                NOTHING = 99
            };

            public:
            ysCamera3DSolver();

            ~ysCamera3DSolver() {
                delete ys_eye_fk_solver_;
                delete ys_eye_tracik_solver_;
            }


        private:
            void subCommandStateCB(const std_msgs::msg::Int32 cmd);
            void ys_subJointStateCB(const sensor_msgs::msg::JointState state);
            void ys_subEyePointCloudCB(const sensor_msgs::msg::PointCloud2 msg);
            
            bool ys_doPCLTemplateAlign();

        private:
            //app
            int app_cmd_;
            std::string template_pathname_;
            //ys robot
            int joint_size_;
            std::vector<std::string> joint_names_;
            std::string ys_prefix_;
            //robot data
            KDL::JntArray ys_cur_q_;
            KDL::Frame ys_curP_eye_;
            bool ys_first_q_;//connected or not
            //camera data
            int try_count_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr eye_cloud_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr base_cloud_;

            //fk ik
            KDL::JntArray ys_max_jnt_;
            KDL::JntArray ys_min_jnt_;
            KDL::JntArray ys_vel_limit_;
            KDL::Chain ys_eye_chain_;
            KDL::ChainFkSolverPos_recursive *ys_eye_fk_solver_;
            TRAC_IK::TRAC_IK *ys_eye_tracik_solver_;

            //topic and service
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_; 
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ys_jointstates_sub_;      
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ys_pointcloud2_sub_;     
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ys_polish_pose_publisher_;  

        };
    }
}



#endif // ysCamera3DSolver_HPP
