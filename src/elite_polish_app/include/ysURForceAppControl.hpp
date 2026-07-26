#ifndef ysUR_ForceAppControl_HPP
#define ysUR_ForceAppControl_HPP

#include <vector>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trac_ik/trac_ik.hpp>

namespace elite_robot {
    namespace ur_force_app {

        class ysURForceAppControl  : public rclcpp::Node
        {
            enum AppCommand {
                GO_HOME = 0,
                DO_AGV_GO_HOME = 1,
                DO_AGV_GO_POLISH = 2,
                DO_CAMERA_VISION_JOB = 3,
                DO_CURVE_POLISHING = 4,
                NOTHING = 99
            };

        public:
            ysURForceAppControl();

            ~ysURForceAppControl() {
                delete ys_eye_fk_solver_;
                delete ys_ftsensor_fk_solver_;
                delete ys_tcp_fk_solver_;
                delete ys_tcp_tracik_solver_;
            }

            bool initDataQ();

        private:
            void timer_callback();
            void subCommandStateCB(const std_msgs::msg::Int32 cmd);
            void ys_subJointStateCB(const sensor_msgs::msg::JointState state);
            void ys_subWrenchCB(const geometry_msgs::msg::WrenchStamped msg);
            void ys_subVisionResultCB(const geometry_msgs::msg::PoseStamped msg);
            void ys_subAGVJobResultCB(const std_msgs::msg::String msg);
            
            KDL::Wrench ys_gravityRepairWrench(const KDL::Wrench &data);

            void calcCurvePolishPath();
            KDL::Frame calcCurvePolishPoint(int stepindex, int yindex);
            KDL::Frame calcSidePolishPoint(int sideindex, int yindex);

            void doAgvGoHome();
            void agv_goHomeCommand();
            void agv_WaitGoHomeDone();

            void doAgvGoPolish();
            void agv_goPolishCommand();
            void agv_WaitGoPolishDone();

            void doVisionJob();
            void vision_goCaptureMove();
            void vision_WaitCaptureMoveDone();
            void vision_SendVisionCmd();
            void vision_WaitVisionSolverDone();

            void doForcePolishing();
            void polish_goPolishBase();
            void polish_waitPolishBase();
            void polish_doForceContact();
            void polish_startPolishtool();
            void polish_doCurvePolishing();
            void polish_endPolishtool();
            void polish_goBackHome();
            void polish_waitBackHome();

            void goHome();
            void goHome_pubMoveHome();
            void goHome_WaitMoveHomeDone();

            void ysPolishTool_Open();
            void ysPolishTool_Close();
        private:
            //app
            int app_cmd_;
            int sub_step_;
            std::mutex ys_mutex_;
            KDL::JntArray ys_home_q_;
            KDL::JntArray ys_cameraCapture_q_;
            KDL::JntArray ys_polishBase_q_;
            int speed_level_;//1,2,3,4

            //force control
            double target_fz_;
            double adjust_dz_;
            int control_dt_count_;//n*4ms for timer
            int control_dt_index_;
            //polish data
            KDL::Frame frame_polishcloud_base_;
            KDL::Frame frame_polishcloud_transform_;
            KDL::Frame frame_forceadjust_base_;
            double polishcurve_radius_;
            double polishcurve_center_dz_;
            double polishcurve_start_ry_;
            double polishcurve_end_ry_;
            double polishproduct_width_;
            int polishcurve_step_count_;
            int polishcurve_step_index_;
            int polishcurve_ycount_;
            int polishcurve_yindex_;
            int sidepolish_step_count_;
            int sidepolish_step_index_;
            std::vector<KDL::Frame> polishcurve_OriginFrames_;
            double dz_polish_startup_tool_;

            //ys robot
            int joint_size_;
            double joint_eps_;
            std::vector<std::string> joint_names_;
            std::string ys_prefix_;
            //robot data
            KDL::JntArray ys_cur_q_;
            KDL::JntArray ys_cur_vel;
            KDL::Frame ys_curP_tcp_;
            KDL::Frame ys_curP_eye_;
            KDL::Frame ys_curP_ftsensor_;
            bool ys_first_q_;//connected or not
            //ftsensor data
            KDL::Wrench ys_origin_wrench_sensor_;
            KDL::Wrench ys_contact_wrench_sensor_;//after gravity and bias fix
            KDL::Wrench ys_contact_wrench_base_;//on base coord
            std::vector<KDL::Wrench> ys_contact_wrench_arr;
            KDL::Wrench ys_average_wrench_;
            int ys_contact_wrench_count;
            int ys_contact_wrench_index;
            bool ys_first_wrench_;//connected or not
            KDL::Vector ys_tool_gravity_;
            KDL::Vector ys_tool_gcenter_;
            KDL::Wrench ys_bias_wrench_;
            //eye
            bool ys_vision_job_done_;
            //agv
            bool agv_go_home_done_;
            bool agv_go_polish_done_;

            //fk ik
            KDL::JntArray ys_max_jnt_;
            KDL::JntArray ys_min_jnt_;
            KDL::JntArray ys_vel_limit_;
            KDL::Chain ys_eye_chain_;
            KDL::ChainFkSolverPos_recursive *ys_eye_fk_solver_;
            KDL::Chain ys_ftsensor_chain_;
            KDL::ChainFkSolverPos_recursive *ys_ftsensor_fk_solver_;
            KDL::Chain ys_tcp_chain_;
            KDL::ChainFkSolverPos_recursive *ys_tcp_fk_solver_;
            TRAC_IK::TRAC_IK *ys_tcp_tracik_solver_;

            //topic and service
            //app cmd
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cmd_sub_; 
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_result_publisher_;  
            //ur
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ys_jointstates_sub_;      
            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ys_traj_publisher_; 
            //ftsensor 
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ys_wrench_sub_;     
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ys_contact_wrench_publisher_;  
            //vision
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vision_job_publisher_;  
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub_; 
            //agv
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr agv_cmd_publisher_;  
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agv_status_sub_;  

            // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ys_polishtool_client_;
            rclcpp::TimerBase::SharedPtr motion_timer_;     
        };
    }
}



#endif // ysUR_ForceAppControl_HPP
