#ifndef ysUR_ForceAppControl_HPP
#define ysUR_ForceAppControl_HPP

#include <chrono>
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
#include "eli_common_interface/srv/force_mode.hpp"
#include "eli_common_interface/srv/set_io.hpp"

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
            void sendEnableForceModeRequest();
            bool disableForceMode();
            void resetForceContactState();
            void publishCurrentPositionHold(double duration_sec);
            double maxJointSpeed() const;
            bool forceModeWatchdog(const KDL::Frame &nominal_frame);

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
            double contact_step_;//402 接触下压每周期步进(m)
            double force_deadband_;//404 力控死区(N)
            int control_dt_index_;
            bool debug_skip_force_contact_ = false;//调试空跑: 402免接触、404力控旁路
            double contact_fz_threshold_ = -16.0;//402接触判定阈值(N)
            int contact_confirm_samples_ = 8;//连续超过阈值的传感器帧数
            int contact_tare_samples_ = 25;//预备姿态局部力零点采样数
            double contact_settle_time_ = 0.3;//预备姿态静置时间(s)
            double contact_hold_time_ = 0.25;//接触后保持位置、等待轨迹停稳时间(s)
            double contact_hold_completion_margin_ = 0.20;//等待action完成回调的额外余量(s)
            double contact_joint_velocity_limit_ = 0.01;//切换力控前最大关节速度(rad/s)
            double contact_max_travel_ = 0.15;//402 沿逼近轴最大行程(m)
            double contact_timeout_ = 90.0;//402 最大逼近时间(s)，150mm@实测约1.9mm/s需约78s
            int contact_ik_max_failures_ = 5;//连续端点IK失败多少周期后中止
            int contact_ik_failure_count_ = 0;
            bool contact_settle_started_ = false;
            std::chrono::steady_clock::time_point contact_settle_start_;
            bool contact_tare_collecting_ = false;
            int contact_tare_count_ = 0;
            double contact_tare_sum_ = 0.0;
            double contact_fz_zero_ = 0.0;//预备姿态的局部力零点
            bool contact_detection_enabled_ = false;
            int contact_confirm_count_ = 0;
            bool contact_confirmed_ = false;
            bool contact_approach_started_ = false;
            std::chrono::steady_clock::time_point contact_approach_start_;
            KDL::Vector contact_approach_start_p_;
            KDL::Vector contact_approach_axis_base_{0.0, 0.0, 1.0};
            bool contact_hold_started_ = false;
            std::chrono::steady_clock::time_point contact_hold_start_;
            bool use_force_mode_ = true;//控制器内建力控(startForceMode)开关
            double force_mode_wrench_z_ = 3.0;//SDK内部目标；+z=世界X+逼近
            double force_mode_z_vel_limit_ = 0.0005;//力控z轴最大调整速度(m/s)
            double force_mode_sensor_target_fz_ = -1.5;//本应用补偿/去皮后的实际反作用力目标(N)
            double force_mode_verify_tolerance_ = 0.6;//启用后实测反作用力与目标的允许误差(N)
            double force_mode_verify_time_ = 0.5;//进入打磨前，目标力连续稳定时间(s)
            double force_mode_verify_timeout_ = 5.0;//启用后建立目标力的最长等待时间(s)
            bool force_mode_verify_stable_ = false;
            std::chrono::steady_clock::time_point force_mode_verify_start_;
            std::chrono::steady_clock::time_point force_mode_verify_stable_start_;
            std::chrono::steady_clock::time_point force_mode_verify_last_log_;
            double force_mode_max_axial_deviation_ = 0.040;//相对名义轨迹最大轴向偏移(m)
            double force_mode_monitor_log_period_ = 1.0;//404 力/轴向补偿监控日志周期(s)
            std::chrono::steady_clock::time_point force_mode_monitor_last_log_;
            double force_mode_abort_fz_ = -5.0;//相对力超过此负值立即退出力控(N)
            double force_mode_min_contact_fz_ = -0.15;//小于此值视为仍有接触(N)
            double force_mode_contact_loss_timeout_ = 2.0;//失去接触允许时间(s)
            KDL::Vector force_mode_axis_base_{0.0, 0.0, 1.0};
            std::chrono::steady_clock::time_point force_mode_last_contact_;
            bool force_mode_enabled_ = false;//当前力控是否已开启
            // 异步使能力控: 单线程 executor 内同步等 future 会死锁(响应处理不到),
            // 改为 async_send_request + 响应回调置 done, 主流程逐拍轮询。
            bool force_mode_enable_pending_ = false;//已发送请求, 等待响应中
            bool force_mode_enable_done_ = false;//响应已收到
            bool force_mode_enable_ok_ = false;//响应结果
            std::chrono::steady_clock::time_point force_mode_enable_start_;//等待超时看门狗
            bool debug_approach_started_ = false;//空跑平滑接近状态
            KDL::JntArray debug_approach_target_q_;//空跑接近目标关节角
            double debug_approach_time_ = 4.0;//空跑接近轨迹时长(s)
            KDL::Vector world_up_in_base_;//退刀: 世界系"上"在 base 系的方向(参数)
            double retract_lift_height_ = 0.15;//退刀: 世界系竖直抬刀高度(m)
            double retract_axial_distance_ = 0.03;//退刀: 先沿接触轴反向离开工件(m)
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
            rclcpp::Client<eli_common_interface::srv::ForceMode>::SharedPtr force_mode_client_;
            rclcpp::Client<eli_common_interface::srv::SetIO>::SharedPtr polish_tool_io_client_;
            //agv
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr agv_cmd_publisher_;  
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr agv_status_sub_;  

            // rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr ys_polishtool_client_;
            rclcpp::TimerBase::SharedPtr motion_timer_;     
        };
    }
}



#endif // ysUR_ForceAppControl_HPP
