/***
***/

#include "rclcpp/rclcpp.hpp" 
#include "ysFTSensorData.hpp"



// ROS2节点主入口main函数
int main(int argc, char * argv[])                      
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                
    
    // 创建ROS2节点对象并进行初始化          
    rclcpp::spin(std::make_shared<ys_ur_robot::ur_force_app::ysFTSensorData>());   
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                

    return 0;
}
