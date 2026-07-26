#ifndef ysFTSensorData_HPP
#define ysFTSensorData_HPP

#include <vector>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace elite_robot {
    namespace ur_force_app {

        class ysFTSensorData  : public rclcpp::Node
        {
        public:
            ysFTSensorData();

            ~ysFTSensorData() {
                delete ys_ftsensor_fk_solver_;
            }


        private:
            void ys_subJointStateCB(const sensor_msgs::msg::JointState state);
            void ys_subWrenchCB(const geometry_msgs::msg::WrenchStamped msg);
            
            KDL::Wrench ys_gravityRepairWrench(const KDL::Wrench &data);

        private:
            //ys robot
            int joint_size_;
            std::vector<std::string> joint_names_;
            std::string ys_prefix_;
            //robot data
            KDL::JntArray ys_cur_q_;
            KDL::Frame ys_curP_ftsensor_;
            bool ys_first_q_;//connected or not
            //ftsensor data
            KDL::Wrench ys_contact_wrench_sensor_;//after gravity and bias fix
            std::vector<KDL::Wrench> ys_wrench_base_arr;
            KDL::Wrench ys_average_wrench_base_;
            int ys_wrench_count;
            int ys_wrench_index_;
            bool ys_first_wrench_;//connected or not
            KDL::Vector ys_tool_gravity_;
            KDL::Vector ys_tool_gcenter_;
            KDL::Wrench ys_bias_wrench_;

            //fk 
            KDL::JntArray ys_max_jnt_;
            KDL::JntArray ys_min_jnt_;
            KDL::JntArray ys_vel_limit_;
            KDL::Chain ys_ftsensor_chain_;
            KDL::ChainFkSolverPos_recursive *ys_ftsensor_fk_solver_;

            //topic and service
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ys_jointstates_sub_;      
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ys_wrench_sub_;     
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ys_contact_wrench_publisher_;  

        };
    }
}



#endif // ysFTSensorData_HPP
